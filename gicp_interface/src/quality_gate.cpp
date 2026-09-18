#include "gicp_interface/quality_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace gicp_localizer {

double hessianConditionProxy(const Eigen::Matrix<double, 6, 6>& hessian) {
  if (!hessian.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  const Eigen::Matrix<double, 6, 6> sym_hessian = 0.5 * (hessian + hessian.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(sym_hessian);
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }

  const auto abs_eigenvalues = solver.eigenvalues().cwiseAbs();
  const double max_eigenvalue = abs_eigenvalues.maxCoeff();
  const double min_eigenvalue = abs_eigenvalues.minCoeff();

  // [P2 FIX 2026-07-10f] Rank deficiency must fail CLOSED. The old code
  // EXCLUDED eigenvalues <= 1e-12 from the min and took the ratio over the
  // survivors: a spectrum like [0, 1e9, 1e9, 1e9, 1e9, 1e9] — one axis with
  // literally no correspondence support — reported condition ~= 1, so the
  // degeneracy projection and the binary Hessian gate stayed INACTIVE on the
  // exact frames they exist to protect (GICP hessian magnitudes run ~1e9, so
  // the 1e-12 floor only ever matched structural zeros). Any near-zero
  // eigenvalue now yields +inf = worst possible conditioning, which both the
  // partial-update trigger and the legacy binary gate treat as degenerate
  // (they gate on `condition > threshold`; +inf trips every finite
  // threshold, and the eigen-projection then keeps the IMU prior along the
  // null directions).
  if (!std::isfinite(max_eigenvalue) || max_eigenvalue <= 0.0 || min_eigenvalue <= 1e-12) {
    return std::numeric_limits<double>::infinity();
  }

  return max_eigenvalue / min_eigenvalue;
}

namespace {

double scalarMarginalStiffness(
    const Eigen::Matrix<double, 6, 6>& hessian,
    int selected_axis) {
  int other_axes[5];
  int next = 0;
  for (int axis = 0; axis < 6; ++axis) {
    if (axis != selected_axis) {
      other_axes[next++] = axis;
    }
  }

  Eigen::Matrix<double, 5, 5> nuisance_hessian;
  Eigen::Matrix<double, 5, 1> coupling;
  for (int row = 0; row < 5; ++row) {
    coupling(row) = hessian(other_axes[row], selected_axis);
    for (int col = 0; col < 5; ++col) {
      nuisance_hessian(row, col) =
          hessian(other_axes[row], other_axes[col]);
    }
  }
  nuisance_hessian =
      0.5 * (nuisance_hessian + nuisance_hessian.transpose());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 5, 5>> solver(
      nuisance_hessian);
  if (solver.info() != Eigen::Success) {
    return -1.0;
  }
  const Eigen::Matrix<double, 5, 1> eigenvalues = solver.eigenvalues();
  const double largest = eigenvalues.cwiseAbs().maxCoeff();
  if (!std::isfinite(largest) || largest <= 0.0) {
    return -1.0;
  }
  const double tolerance = std::max(1e-12, largest * 1e-12);
  Eigen::Matrix<double, 5, 1> nuisance_solution =
      Eigen::Matrix<double, 5, 1>::Zero();
  for (int i = 0; i < 5; ++i) {
    const double eigenvalue = eigenvalues(i);
    if (eigenvalue < -tolerance) {
      return -1.0;
    }
    if (eigenvalue > tolerance) {
      const auto direction = solver.eigenvectors().col(i);
      nuisance_solution +=
          direction * (direction.dot(coupling) / eigenvalue);
    }
  }

  const double stiffness =
      hessian(selected_axis, selected_axis) -
      coupling.dot(nuisance_solution);
  const double scale =
      std::max(1.0, std::abs(hessian(selected_axis, selected_axis)));
  if (!std::isfinite(stiffness) || stiffness < -1e-9 * scale) {
    return -1.0;
  }
  return std::max(0.0, stiffness);
}

}  // namespace

TrackHessianInformation trackDirectionalHessianInformation(
    const Eigen::Matrix<double, 6, 6>& hessian,
    const Eigen::Vector3d& vehicle_position,
    double track_heading_rad) {
  TrackHessianInformation information;
  if (!hessian.allFinite() || !vehicle_position.allFinite() ||
      !std::isfinite(track_heading_rad)) {
    return information;
  }

  const Eigen::Matrix<double, 6, 6> symmetric_hessian =
      0.5 * (hessian + hessian.transpose());
  Eigen::Matrix3d skew_position;
  skew_position << 0.0, -vehicle_position.z(), vehicle_position.y(),
      vehicle_position.z(), 0.0, -vehicle_position.x(),
      -vehicle_position.y(), vehicle_position.x(), 0.0;
  Eigen::Matrix<double, 6, 6> recenter =
      Eigen::Matrix<double, 6, 6>::Identity();
  recenter.block<3, 3>(3, 0) = skew_position;
  const Eigen::Matrix<double, 6, 6> vehicle_hessian =
      recenter.transpose() * symmetric_hessian * recenter;

  const double cosine = std::cos(track_heading_rad);
  const double sine = std::sin(track_heading_rad);
  Eigen::Matrix3d track_basis;
  track_basis << cosine, -sine, 0.0,
                 sine,  cosine, 0.0,
                 0.0,   0.0,   1.0;
  Eigen::Matrix<double, 6, 6> basis =
      Eigen::Matrix<double, 6, 6>::Identity();
  basis.block<3, 3>(3, 3) = track_basis;
  const Eigen::Matrix<double, 6, 6> track_hessian =
      basis.transpose() * vehicle_hessian * basis;

  information.tangent_marginal_stiffness =
      scalarMarginalStiffness(track_hessian, 3);
  information.normal_marginal_stiffness =
      scalarMarginalStiffness(track_hessian, 4);
  return information;
}

// ---------------------------------------------------------------------------
// P1 gating rework: degeneracy-aware partial update ("solution remapping",
// Zhang & Singh ICRA'16). Instead of binary-rejecting a scan whose hessian is
// ill-conditioned, project the GICP correction onto the well-constrained
// eigen-subspace and keep the IMU prior along the degenerate directions.
//
// Frame handling: small_gicp's final hessian is parameterized as [omega; t]
// with the rotation taken about the WORLD ORIGIN (jacobian block is
// skew(transformed_point)). With the vehicle ~hundreds of metres from the map
// origin, a yaw about the origin is numerically indistinguishable from a
// translation, so the raw rotation block mostly measures lever-arm effects.
// We therefore re-center the hessian about the vehicle position c first:
// with new variables [omega; t_hat], t_hat = t + omega x c, the substitution
// x = A x_hat, A = [[I,0],[skew(c),I]] gives H_c = A^T H A whose rotation
// block measures rotations ABOUT THE VEHICLE. Conveniently t_hat is, to first
// order, exactly candidate_p - prior_p, so the projected translation applies
// directly to the pose difference.
// ---------------------------------------------------------------------------

DegeneracyProjection projectDegenerateDelta(const Eigen::Matrix<double, 6, 6>& hessian,
                                            const Eigen::Matrix4f& T_prior,
                                            const Eigen::Matrix4f& candidate,
                                            bool apply_eigen_projection,
                                            bool full6d,
                                            double coupling_length_m,
                                            double rel_floor_6d,
                                            double rel_floor_rot,
                                            double rel_floor_trans,
                                            double veto_yaw_above_deg /* <=0 disables */,
                                            double clamp_rp_above_deg /* <=0 disables */) {
  DegeneracyProjection out;
  out.projected_pose = candidate;
  // A non-finite hessian only invalidates the EIGEN projection — the yaw veto
  // works on the pose delta alone and must not be silently disabled by it
  // (review fix). Callers treat valid=false as "reject when eigen projection
  // was required", which is still the right contract below.
  if (apply_eigen_projection && !hessian.allFinite()) return out;

  const Eigen::Matrix4d prior = T_prior.cast<double>();
  const Eigen::Matrix4d cand = candidate.cast<double>();
  const Eigen::Matrix3d R_prior = prior.block<3, 3>(0, 0);
  const Eigen::Matrix3d R_cand = cand.block<3, 3>(0, 0);
  const Eigen::Vector3d p_prior = prior.block<3, 1>(0, 3);
  const Eigen::Vector3d p_cand = cand.block<3, 1>(0, 3);

  // World-frame delta: candidate = T_delta * prior.
  const Eigen::Matrix3d R_delta = R_cand * R_prior.transpose();
  Eigen::AngleAxisd aa(R_delta);
  Eigen::Vector3d omega = aa.angle() * aa.axis();   // rotation correction (world axes, about vehicle)
  Eigen::Vector3d t_hat = p_cand - p_prior;         // translation correction of the vehicle

  Eigen::Vector3d omega_p = omega;
  Eigen::Vector3d t_hat_p = t_hat;

  if (apply_eigen_projection) {
    // Re-center the hessian about the vehicle position.
    const Eigen::Matrix<double, 6, 6> H_sym = 0.5 * (hessian + hessian.transpose());
    Eigen::Matrix3d skew_c;
    skew_c << 0.0, -p_prior.z(), p_prior.y(),
              p_prior.z(), 0.0, -p_prior.x(),
             -p_prior.y(), p_prior.x(), 0.0;
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
    A.block<3, 3>(3, 0) = skew_c;
    const Eigen::Matrix<double, 6, 6> H_c = A.transpose() * H_sym * A;

    if (full6d) {
      // Full 6D solution remapping (Zhang & Singh). Rotation (rad) and
      // translation (m) are incommensurable, so scale rotation coordinates by
      // a characteristic coupling length L first: x_s = [L*omega; t_hat],
      // x = D x_s with D = diag(I/L, I), H_s = D H_c D. L should be the
      // typical constraint lever arm (~point-cloud radius after the 80 m
      // crop); with it, one unit of any scaled coordinate moves constraint
      // points by comparable metres, making the joint spectrum meaningful and
      // COUPLED rot/trans null directions (e.g. slide-along-a-wall = yaw +
      // lateral mix) visible — a blockwise analysis structurally cannot see
      // those.
      const double L = std::max(coupling_length_m, 1e-3);
      Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Identity();
      D.block<3, 3>(0, 0) /= L;
      const Eigen::Matrix<double, 6, 6> H_s = D * H_c * D;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H_s);
      if (es.info() != Eigen::Success) {
        return out;  // eigensolver failure: leave candidate untouched, valid=false
      }
      const Eigen::Matrix<double, 6, 1> evals = es.eigenvalues().cwiseAbs();
      const double lambda_max = evals.maxCoeff();
      Eigen::Matrix<double, 6, 6> P6 = Eigen::Matrix<double, 6, 6>::Zero();
      int n_degen = 0;
      for (int i = 0; i < 6; ++i) {
        const Eigen::Matrix<double, 6, 1> v = es.eigenvectors().col(i);
        if (lambda_max <= 0.0 || evals[i] < rel_floor_6d * lambda_max) {
          ++n_degen;
          // Report which sub-block the degenerate direction mostly lives in
          // (diagnostic only; the projector itself is fully coupled).
          if (v.head<3>().norm() >= v.tail<3>().norm()) {
            ++out.degen_rot_axes;
          } else {
            ++out.degen_trans_axes;
          }
        } else {
          P6 += v * v.transpose();
        }
      }
      if (n_degen == 6) {
        out.valid = true;
        out.fully_degenerate = true;
        return out;
      }
      Eigen::Matrix<double, 6, 1> dx_s;
      dx_s.head<3>() = L * omega;
      dx_s.tail<3>() = t_hat;
      const Eigen::Matrix<double, 6, 1> dx_s_p = P6 * dx_s;
      omega_p = dx_s_p.head<3>() / L;
      t_hat_p = dx_s_p.tail<3>();
    } else {
      // Blockwise fallback (degeneracy/full6d: false): independent 3x3
      // eigen-analyses of the rot/trans blocks. Unit-mixing-free but blind to
      // coupled rot/trans degeneracy; kept for A/B comparison.
      Eigen::Matrix3d P_rot = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d P_trans = Eigen::Matrix3d::Identity();
      auto blockProjector = [](const Eigen::Matrix3d& block, double rel_floor,
                               Eigen::Matrix3d& projector, int& n_degen) -> bool {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(block);
        if (es.info() != Eigen::Success) return false;
        const Eigen::Vector3d evals = es.eigenvalues().cwiseAbs();
        const double lambda_max = evals.maxCoeff();
        projector.setZero();
        n_degen = 0;
        for (int i = 0; i < 3; ++i) {
          if (lambda_max <= 0.0 || evals[i] < rel_floor * lambda_max) {
            ++n_degen;
          } else {
            const Eigen::Vector3d v = es.eigenvectors().col(i);
            projector += v * v.transpose();
          }
        }
        return true;
      };

      if (!blockProjector(H_c.block<3, 3>(0, 0), rel_floor_rot, P_rot, out.degen_rot_axes) ||
          !blockProjector(H_c.block<3, 3>(3, 3), rel_floor_trans, P_trans, out.degen_trans_axes)) {
        return out;  // eigensolver failure: leave candidate untouched, valid=false
      }
      if (out.degen_rot_axes == 3 && out.degen_trans_axes == 3) {
        out.valid = true;
        out.fully_degenerate = true;
        return out;
      }
      omega_p = P_rot * omega;
      t_hat_p = P_trans * t_hat;
    }
  }

  // Turn-aware yaw-consistency veto: T_prior already contains the
  // IMU-integrated yaw across the scan gap, so omega.z() IS the GICP-vs-IMU
  // yaw disagreement. A large disagreement on a low-confidence match is the
  // wrong-basin entry signature; keep the IMU yaw instead.
  constexpr double kRad2Deg = 180.0 / M_PI;
  if (veto_yaw_above_deg > 0.0 && std::abs(omega_p.z()) * kRad2Deg > veto_yaw_above_deg) {
    omega_p.z() = 0.0;
    out.yaw_vetoed = true;
  }

  // [P2 FIX 2026-07-10h] Roll/pitch innovation clamp — the attitude analogue
  // of the hard yaw veto. On the periodic full-6DoF re-anchor scans
  // (gicp/dof/full6dofEveryN) BOTH attitude priors are typically zero and the
  // generic rotation jump gate is ~30 deg + time scaling: erroneous roll/
  // pitch from repeated map structure could pass unopposed even though yaw
  // has its dedicated 8 deg guard. Gyro roll/pitch drift over one scan gap is
  // <<1 deg, so a large rp innovation is as physically impossible as a large
  // yaw one. CLAMP (scale down) rather than zero: the 6-DoF refresh exists
  // precisely to let roll/pitch re-anchor by SMALL amounts. Inert on 4-DoF
  // scans (exact mask keeps the rp delta at zero).
  if (clamp_rp_above_deg > 0.0) {
    const double rp_deg =
        std::sqrt(omega_p.x() * omega_p.x() + omega_p.y() * omega_p.y()) * kRad2Deg;
    if (rp_deg > clamp_rp_above_deg) {
      const double scale = clamp_rp_above_deg / rp_deg;
      omega_p.x() *= scale;
      omega_p.y() *= scale;
      out.rp_clamped = true;
    }
  }

  const double kEps = 1e-12;
  const bool changed = ((omega_p - omega).norm() > kEps) || ((t_hat_p - t_hat).norm() > kEps);
  out.valid = true;
  if (changed) {
    Eigen::Matrix3d R_delta_p = Eigen::Matrix3d::Identity();
    const double angle = omega_p.norm();
    if (angle > kEps) {
      R_delta_p = Eigen::AngleAxisd(angle, omega_p / angle).toRotationMatrix();
    }
    Eigen::Matrix4d projected = Eigen::Matrix4d::Identity();
    projected.block<3, 3>(0, 0) = R_delta_p * R_prior;
    projected.block<3, 1>(0, 3) = p_prior + t_hat_p;
    out.projected_pose = projected.cast<float>();
    out.modified = true;
  }
  return out;
}

// P1 yaw-safety: yaw component (deg, absolute) of the world-frame delta
// between two poses — the z element of the rotation vector of
// R_b * R_a^T. For a near-level ground vehicle this is the heading
// disagreement; roll/pitch live in the x/y components and are gated
// separately by the total-rotation jump threshold.
double yawInnovationDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const Eigen::Matrix3d Ra = a.block<3, 3>(0, 0).cast<double>();
  const Eigen::Matrix3d Rb = b.block<3, 3>(0, 0).cast<double>();
  const Eigen::AngleAxisd aa(Rb * Ra.transpose());
  const Eigen::Vector3d rv = aa.angle() * aa.axis();
  return std::abs(rv.z()) * 180.0 / M_PI;
}

RegistrationGateDecision evaluateRegistrationGate(
    const RegistrationGateConfig& config,
    const RegistrationGateInput& input) {
  RegistrationGateDecision decision;
  if (!input.effectively_converged || !input.candidate_pose_valid) {
    return decision;
  }

  if (!input.hessian_finite) {
    decision.rejected_hessian = true;
  } else if ((config.min_correspondences > 0 &&
              input.support_correspondences < config.min_correspondences) ||
             (config.min_correspondence_ratio > 0.0 &&
              input.support_ratio < config.min_correspondence_ratio)) {
    decision.rejected_support = true;
  } else if (!std::isfinite(input.fitness) ||
             input.fitness > config.fitness_reject_threshold) {
    decision.rejected_fitness = true;
  } else if (input.yaw_jump) {
    decision.rejected_yaw = true;
  } else if (config.fitness_ratio_reject_threshold > 0.0 &&
             input.fitness_ratio > 0.0 &&
             input.fitness_ratio > config.fitness_ratio_reject_threshold) {
    decision.rejected_fitness_ratio = true;
  } else if (config.degeneracy_partial_update) {
    if (input.eigen_projection_wanted &&
        (!input.degeneracy_projection_valid || input.fully_degenerate)) {
      decision.rejected_hessian = true;
    } else if (config.reject_large_jumps && input.large_jump) {
      decision.rejected_jump = true;
    }
  } else if (config.hessian_condition_max > 0.0 &&
             input.hessian_finite &&
             input.hessian_condition > config.hessian_condition_max &&
             ((config.hessian_fitness_warn > 0.0 &&
               input.fitness > config.hessian_fitness_warn) ||
              (config.hessian_translation_warn_m > 0.0 &&
               input.solution_translation_m > config.hessian_translation_warn_m) ||
              (config.hessian_rotation_warn_deg > 0.0 &&
               input.solution_rotation_deg > config.hessian_rotation_warn_deg) ||
              (config.hessian_fitness_warn <= 0.0 &&
               config.hessian_translation_warn_m <= 0.0 &&
               config.hessian_rotation_warn_deg <= 0.0))) {
    decision.rejected_hessian = true;
  } else if (config.reject_large_jumps && input.large_jump) {
    decision.rejected_jump = true;
  }

  decision.accepted =
      !decision.rejected_fitness &&
      !decision.rejected_fitness_ratio &&
      !decision.rejected_jump &&
      !decision.rejected_yaw &&
      !decision.rejected_hessian &&
      !decision.rejected_support;
  return decision;
}

}  // namespace gicp_localizer
