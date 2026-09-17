#pragma once

#include <Eigen/Core>

namespace gicp_localizer {

struct DegeneracyProjection {
  bool valid{false};
  bool modified{false};
  bool fully_degenerate{false};
  bool yaw_vetoed{false};
  bool rp_clamped{false};
  int degen_rot_axes{0};
  int degen_trans_axes{0};
  Eigen::Matrix4f projected_pose = Eigen::Matrix4f::Identity();
};

double hessianConditionProxy(const Eigen::Matrix<double, 6, 6>& hessian);

struct TrackHessianInformation {
  double tangent_marginal_stiffness{-1.0};
  double normal_marginal_stiffness{-1.0};

  bool valid() const {
    return tangent_marginal_stiffness >= 0.0 &&
           normal_marginal_stiffness >= 0.0;
  }
};

// Report the scan-only translation information along and across the local
// track heading after rotation, vertical translation, and the other planar
// direction have been marginalized. A negative value means unavailable.
TrackHessianInformation trackDirectionalHessianInformation(
    const Eigen::Matrix<double, 6, 6>& hessian,
    const Eigen::Vector3d& vehicle_position,
    double track_heading_rad);

DegeneracyProjection projectDegenerateDelta(
    const Eigen::Matrix<double, 6, 6>& hessian,
    const Eigen::Matrix4f& T_prior,
    const Eigen::Matrix4f& candidate,
    bool apply_eigen_projection,
    bool full6d,
    double coupling_length_m,
    double rel_floor_6d,
    double rel_floor_rot,
    double rel_floor_trans,
    double veto_yaw_above_deg,
    double clamp_rp_above_deg);

double yawInnovationDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b);

struct RegistrationGateConfig {
  int min_correspondences{0};
  double min_correspondence_ratio{0.0};
  double fitness_reject_threshold{0.0};
  double fitness_ratio_reject_threshold{0.0};
  bool degeneracy_partial_update{true};
  bool reject_large_jumps{true};
  double hessian_condition_max{0.0};
  double hessian_fitness_warn{0.0};
  double hessian_translation_warn_m{0.0};
  double hessian_rotation_warn_deg{0.0};
};

struct RegistrationGateInput {
  bool effectively_converged{false};
  bool candidate_pose_valid{false};
  bool hessian_finite{false};
  int support_correspondences{0};
  double support_ratio{0.0};
  double fitness{0.0};
  bool yaw_jump{false};
  double fitness_ratio{0.0};
  bool eigen_projection_wanted{false};
  bool degeneracy_projection_valid{false};
  bool fully_degenerate{false};
  bool large_jump{false};
  double hessian_condition{0.0};
  double solution_translation_m{0.0};
  double solution_rotation_deg{0.0};
};

struct RegistrationGateDecision {
  bool accepted{false};
  bool rejected_fitness{false};
  bool rejected_fitness_ratio{false};
  bool rejected_jump{false};
  bool rejected_yaw{false};
  bool rejected_hessian{false};
  bool rejected_support{false};
};

RegistrationGateDecision evaluateRegistrationGate(
    const RegistrationGateConfig& config,
    const RegistrationGateInput& input);

}  // namespace gicp_localizer
