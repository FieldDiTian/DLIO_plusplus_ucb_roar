#pragma once

#include "gicp_interface/detail/voxel_surfel_map.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gicp_localizer::detail {

struct LidarObservation {
  Eigen::Vector3d point_body{Eigen::Vector3d::Zero()};
  std::uint8_t sensor_id{0};
};

struct PlanarResidual {
  Eigen::Vector3d point_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jacobian{Eigen::Vector3d::Zero()};
  double residual_m{0.0};
  double robust_weight{1.0};
  std::uint8_t sensor_id{0};
};

struct PlanarMeasurementConfig {
  double maximum_plane_distance_m{0.5};
  double maximum_centroid_distance_m{1.0};
  double huber_delta_m{0.15};
};

struct PlanarNormalEquations {
  Eigen::Matrix3d hessian{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d gradient{Eigen::Vector3d::Zero()};
  double robust_cost{0.0};
  std::size_t correspondence_count{0};
};

std::vector<PlanarResidual> build_planar_residuals(
    const VoxelSurfelMap &map, const Eigen::Isometry3d &world_from_body,
    const std::vector<LidarObservation> &observations,
    const PlanarMeasurementConfig &config = PlanarMeasurementConfig{});

PlanarNormalEquations accumulate_planar_normal_equations(
    const std::vector<PlanarResidual> &residuals,
    const std::vector<std::size_t> &selected_indices = {});

Eigen::Vector3d solve_planar_increment(const PlanarNormalEquations &equations,
                                       double diagonal_damping = 1e-6);

// Translation information resolved along the body's planar heading (x), its
// left normal (y), and yaw (z). This is diagnostic only and does not rotate or
// otherwise modify the normal equations used by the estimator.
Eigen::Vector3d heading_frame_information(const Eigen::Matrix3d &hessian,
                                          double yaw_rad);

} // namespace gicp_localizer::detail
