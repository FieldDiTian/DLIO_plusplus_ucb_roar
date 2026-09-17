#pragma once

#include "gicp_interface/detail/planar_measurement.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace gicp_localizer::detail {

constexpr int kSpatialPoseDimension = 6;
using SpatialVector = Eigen::Matrix<double, kSpatialPoseDimension, 1>;
using SpatialMatrix =
    Eigen::Matrix<double, kSpatialPoseDimension, kSpatialPoseDimension>;

struct SpatialResidual {
  Eigen::Vector3d point_world{Eigen::Vector3d::Zero()};
  SpatialVector jacobian{SpatialVector::Zero()};
  double residual_m{0.0};
  double robust_weight{1.0};
  std::uint8_t sensor_id{0};
};

struct SpatialNormalEquations {
  SpatialMatrix hessian{SpatialMatrix::Zero()};
  SpatialVector gradient{SpatialVector::Zero()};
  double robust_cost{0.0};
  std::size_t correspondence_count{0};
};

std::vector<SpatialResidual> build_spatial_residuals(
    const VoxelSurfelMap &map, const Eigen::Isometry3d &world_from_body,
    const std::vector<LidarObservation> &observations,
    const PlanarMeasurementConfig &config = PlanarMeasurementConfig{});

SpatialNormalEquations accumulate_spatial_normal_equations(
    const std::vector<SpatialResidual> &residuals,
    const std::vector<std::size_t> &selected_indices = {});

SpatialVector solve_spatial_increment(const SpatialNormalEquations &equations,
                                      double diagonal_damping = 1e-6);

// Eliminate the scan-local [z, roll, pitch] nuisance increment while retaining
// all six point-to-plane Jacobian columns. The returned equations act only on
// persistent [x, y, yaw]. nuisance_residual is ordered [z, roll, pitch] and is
// regularized by the corresponding independent variances.
PlanarNormalEquations marginalize_spatial_nuisance(
    const SpatialNormalEquations &equations,
    const Eigen::Vector3d &nuisance_residual,
    const Eigen::Vector3d &nuisance_variance,
    double diagonal_damping = 1e-9);

// Extract the [world x, world y, world yaw] information block so the existing
// race-track observability diagnostic remains directly comparable with the
// frozen planar baseline.
Eigen::Matrix3d planar_information_block(const SpatialMatrix &hessian);

} // namespace gicp_localizer::detail
