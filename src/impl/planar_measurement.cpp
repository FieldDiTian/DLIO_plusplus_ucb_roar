#include "gicp_interface/detail/planar_measurement.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>

namespace gicp_localizer::detail {
namespace {

double huber_weight(double residual_m, double delta_m) {
  const double magnitude = std::abs(residual_m);
  return magnitude <= delta_m ? 1.0 : delta_m / magnitude;
}

double huber_cost(double residual_m, double delta_m) {
  const double magnitude = std::abs(residual_m);
  return magnitude <= delta_m ? 0.5 * residual_m * residual_m
                              : delta_m * (magnitude - 0.5 * delta_m);
}

void validate_config(const PlanarMeasurementConfig &config) {
  if (!std::isfinite(config.maximum_plane_distance_m) ||
      !std::isfinite(config.maximum_centroid_distance_m) ||
      !std::isfinite(config.huber_delta_m) ||
      config.maximum_plane_distance_m <= 0.0 ||
      config.maximum_centroid_distance_m <= 0.0 ||
      config.huber_delta_m <= 0.0) {
    throw std::invalid_argument("invalid planar measurement configuration");
  }
}

} // namespace

std::vector<PlanarResidual>
build_planar_residuals(const VoxelSurfelMap &map,
                       const Eigen::Isometry3d &world_from_body,
                       const std::vector<LidarObservation> &observations,
                       const PlanarMeasurementConfig &config) {
  if (!world_from_body.matrix().allFinite()) {
    throw std::invalid_argument("pose must be finite");
  }
  validate_config(config);

  std::vector<PlanarResidual> residuals;
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
    const Eigen::Vector3d yaw_derivative =
        Eigen::Vector3d::UnitZ().cross(rotated_point);
    const Eigen::Vector3d jacobian(normal.x(), normal.y(),
                                   normal.dot(yaw_derivative));
    residuals.push_back(PlanarResidual{
        point_world, jacobian, match->signed_plane_distance_m,
        huber_weight(match->signed_plane_distance_m, config.huber_delta_m),
        observation.sensor_id});
  }
  return residuals;
}

PlanarNormalEquations accumulate_planar_normal_equations(
    const std::vector<PlanarResidual> &residuals,
    const std::vector<std::size_t> &selected_indices) {
  PlanarNormalEquations equations;
  const auto accumulate = [&equations, &residuals](std::size_t index) {
    if (index >= residuals.size()) {
      throw std::out_of_range("selected residual index is out of range");
    }
    const PlanarResidual &residual = residuals[index];
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
    for (std::size_t i = 0; i < residuals.size(); ++i) {
      accumulate(i);
    }
  } else {
    for (const std::size_t index : selected_indices) {
      accumulate(index);
    }
  }
  return equations;
}

Eigen::Vector3d solve_planar_increment(const PlanarNormalEquations &equations,
                                       double diagonal_damping) {
  if (!equations.hessian.allFinite() || !equations.gradient.allFinite() ||
      !std::isfinite(diagonal_damping) || diagonal_damping < 0.0) {
    throw std::invalid_argument("invalid planar normal equations");
  }
  const Eigen::Matrix3d damped =
      equations.hessian + diagonal_damping * Eigen::Matrix3d::Identity();
  const Eigen::LDLT<Eigen::Matrix3d> factorization(damped);
  if (factorization.info() != Eigen::Success || !factorization.isPositive()) {
    throw std::runtime_error(
        "planar normal equations are not positive definite");
  }
  const Eigen::Vector3d increment = factorization.solve(-equations.gradient);
  if (!increment.allFinite()) {
    throw std::runtime_error("planar solve produced non-finite increment");
  }
  return increment;
}

Eigen::Vector3d heading_frame_information(const Eigen::Matrix3d &hessian,
                                          double yaw_rad) {
  if (!hessian.allFinite() || !std::isfinite(yaw_rad)) {
    throw std::invalid_argument("invalid planar information projection");
  }
  const Eigen::Vector3d tangent(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);
  const Eigen::Vector3d normal(-std::sin(yaw_rad), std::cos(yaw_rad), 0.0);
  return Eigen::Vector3d(tangent.dot(hessian * tangent),
                         normal.dot(hessian * normal), hessian(2, 2));
}

} // namespace gicp_localizer::detail
