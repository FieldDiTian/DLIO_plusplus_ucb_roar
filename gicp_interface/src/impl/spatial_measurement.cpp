#include "gicp_interface/detail/spatial_measurement.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>

namespace gicp_localizer::detail {
namespace {

double huber_weight(double residual_m, double delta_m) {
  const double magnitude = std::abs(residual_m);
  return magnitude <= delta_m ? 1.0 : delta_m / magnitude;
}

void validate_config(const PlanarMeasurementConfig &config) {
  if (!std::isfinite(config.maximum_plane_distance_m) ||
      !std::isfinite(config.maximum_centroid_distance_m) ||
      !std::isfinite(config.huber_delta_m) ||
      config.maximum_plane_distance_m <= 0.0 ||
      config.maximum_centroid_distance_m <= 0.0 ||
      config.huber_delta_m <= 0.0) {
    throw std::invalid_argument("invalid spatial measurement configuration");
  }
}

} // namespace

std::vector<SpatialResidual>
build_spatial_residuals(const VoxelSurfelMap &map,
                        const Eigen::Isometry3d &world_from_body,
                        const std::vector<LidarObservation> &observations,
                        const PlanarMeasurementConfig &config) {
  if (!world_from_body.matrix().allFinite()) {
    throw std::invalid_argument("pose must be finite");
  }
  validate_config(config);

  std::vector<SpatialResidual> residuals;
  residuals.reserve(observations.size());
  for (const LidarObservation &observation : observations) {
    if (!observation.point_body.allFinite()) {
      continue;
    }
    const Eigen::Vector3d rotated_point =
        world_from_body.linear() * observation.point_body;
    const Eigen::Vector3d point_world =
        rotated_point + world_from_body.translation();
    const auto match =
        map.nearest_surfel(point_world, config.maximum_plane_distance_m,
                           config.maximum_centroid_distance_m);
    if (!match.has_value()) {
      continue;
    }

    const Eigen::Vector3d &normal = match->surfel->normal;
    SpatialVector jacobian;
    jacobian.head<3>() = normal;
    // The filter applies a left-multiplicative world-frame rotation error:
    // Exp(delta_theta) * R. Therefore d(Rp)/d(delta_theta) = -[Rp]_x.
    jacobian.tail<3>() = rotated_point.cross(normal);
    residuals.push_back(SpatialResidual{
        point_world, jacobian, match->signed_plane_distance_m,
        huber_weight(match->signed_plane_distance_m, config.huber_delta_m),
        observation.sensor_id});
  }
  return residuals;
}

SpatialNormalEquations accumulate_spatial_normal_equations(
    const std::vector<SpatialResidual> &residuals,
    const std::vector<std::size_t> &selected_indices) {
  SpatialNormalEquations equations;
  const auto accumulate = [&equations, &residuals](std::size_t index) {
    if (index >= residuals.size()) {
      throw std::out_of_range("selected residual index is out of range");
    }
    const SpatialResidual &residual = residuals[index];
    if (!residual.jacobian.allFinite() || !std::isfinite(residual.residual_m) ||
        !std::isfinite(residual.robust_weight) ||
        residual.robust_weight < 0.0) {
      throw std::invalid_argument(
          "residual must be finite with non-negative weight");
    }
    equations.hessian.noalias() += residual.robust_weight * residual.jacobian *
                                   residual.jacobian.transpose();
    equations.gradient.noalias() +=
        residual.robust_weight * residual.jacobian * residual.residual_m;
    equations.robust_cost +=
        residual.robust_weight * residual.residual_m * residual.residual_m;
    ++equations.correspondence_count;
  };

  if (selected_indices.empty()) {
    for (std::size_t index = 0; index < residuals.size(); ++index) {
      accumulate(index);
    }
  } else {
    for (const std::size_t index : selected_indices) {
      accumulate(index);
    }
  }
  return equations;
}

SpatialVector solve_spatial_increment(const SpatialNormalEquations &equations,
                                      double diagonal_damping) {
  if (!equations.hessian.allFinite() || !equations.gradient.allFinite() ||
      !std::isfinite(diagonal_damping) || diagonal_damping < 0.0) {
    throw std::invalid_argument("invalid spatial normal equations");
  }
  const SpatialMatrix damped =
      equations.hessian + diagonal_damping * SpatialMatrix::Identity();
  const Eigen::LDLT<SpatialMatrix> factorization(damped);
  if (factorization.info() != Eigen::Success || !factorization.isPositive()) {
    throw std::runtime_error(
        "spatial normal equations are not positive definite");
  }
  const SpatialVector increment = factorization.solve(-equations.gradient);
  if (!increment.allFinite()) {
    throw std::runtime_error("spatial solve produced non-finite increment");
  }
  return increment;
}

PlanarNormalEquations marginalize_spatial_nuisance(
    const SpatialNormalEquations &equations,
    const Eigen::Vector3d &nuisance_residual,
    const Eigen::Vector3d &nuisance_variance, double diagonal_damping) {
  if (!equations.hessian.allFinite() || !equations.gradient.allFinite() ||
      !nuisance_residual.allFinite() || !nuisance_variance.allFinite() ||
      (nuisance_variance.array() <= 0.0).any() ||
      !std::isfinite(diagonal_damping) || diagonal_damping < 0.0) {
    throw std::invalid_argument("invalid spatial nuisance marginalization");
  }

  constexpr int persistent_indices[3] = {0, 1, 5};
  constexpr int nuisance_indices[3] = {2, 3, 4};
  Eigen::Matrix3d persistent_information;
  Eigen::Matrix3d cross_information;
  Eigen::Matrix3d nuisance_information;
  Eigen::Vector3d persistent_gradient;
  Eigen::Vector3d nuisance_gradient;
  for (int row = 0; row < 3; ++row) {
    persistent_gradient(row) = equations.gradient(persistent_indices[row]);
    nuisance_gradient(row) = equations.gradient(nuisance_indices[row]);
    for (int column = 0; column < 3; ++column) {
      persistent_information(row, column) = equations.hessian(
          persistent_indices[row], persistent_indices[column]);
      cross_information(row, column) = equations.hessian(
          persistent_indices[row], nuisance_indices[column]);
      nuisance_information(row, column) = equations.hessian(
          nuisance_indices[row], nuisance_indices[column]);
    }
  }

  const Eigen::Vector3d prior_information = nuisance_variance.cwiseInverse();
  nuisance_information.diagonal() += prior_information;
  nuisance_gradient += prior_information.cwiseProduct(nuisance_residual);
  nuisance_information.diagonal().array() += diagonal_damping;

  const Eigen::LDLT<Eigen::Matrix3d> nuisance_factorization(
      nuisance_information);
  if (nuisance_factorization.info() != Eigen::Success ||
      !nuisance_factorization.isPositive()) {
    throw std::runtime_error(
        "spatial nuisance information is not positive definite");
  }

  PlanarNormalEquations marginalized;
  marginalized.hessian =
      persistent_information -
      cross_information *
          nuisance_factorization.solve(cross_information.transpose());
  marginalized.hessian =
      0.5 * (marginalized.hessian + marginalized.hessian.transpose());
  marginalized.gradient =
      persistent_gradient -
      cross_information * nuisance_factorization.solve(nuisance_gradient);
  marginalized.robust_cost =
      equations.robust_cost +
      prior_information.dot(nuisance_residual.cwiseAbs2());
  marginalized.correspondence_count = equations.correspondence_count;
  if (!marginalized.hessian.allFinite() ||
      !marginalized.gradient.allFinite() ||
      !std::isfinite(marginalized.robust_cost)) {
    throw std::runtime_error(
        "spatial nuisance marginalization produced non-finite equations");
  }
  return marginalized;
}

Eigen::Matrix3d planar_information_block(const SpatialMatrix &hessian) {
  if (!hessian.allFinite()) {
    throw std::invalid_argument("spatial information must be finite");
  }
  constexpr int indices[3] = {0, 1, 5};
  Eigen::Matrix3d planar;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      planar(row, column) = hessian(indices[row], indices[column]);
    }
  }
  return planar;
}

} // namespace gicp_localizer::detail
