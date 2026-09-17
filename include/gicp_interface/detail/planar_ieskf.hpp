#pragma once

#include "gicp_interface/detail/imu_trajectory.hpp"
#include "gicp_interface/detail/planar_measurement.hpp"
#include "gicp_interface/detail/spatial_measurement.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <functional>

namespace gicp_localizer::detail {

constexpr int kErrorStateDimension = 15;
using ErrorCovariance =
    Eigen::Matrix<double, kErrorStateDimension, kErrorStateDimension>;
using ErrorVector = Eigen::Matrix<double, kErrorStateDimension, 1>;

struct ImuNoise {
  double gyro_noise_density{0.002};
  double accel_noise_density{0.05};
  double gyro_bias_random_walk{1e-4};
  double accel_bias_random_walk{1e-3};
};

struct FilterState {
  InertialState nominal;
  ErrorCovariance covariance{ErrorCovariance::Identity()};
};

struct IteratedUpdateConfig {
  std::size_t maximum_iterations{5};
  double translation_convergence_m{1e-3};
  double rotation_convergence_rad{1e-4};
  double minimum_information_eigenvalue{1e-6};
};

struct IteratedUpdateResult {
  std::size_t iterations{0};
  bool converged{false};
  std::size_t correspondence_count{0};
  Eigen::Vector3d planar_increment{Eigen::Vector3d::Zero()};
  Eigen::Vector3d information_eigenvalues{Eigen::Vector3d::Zero()};
  double robust_cost{0.0};
};

struct SpatialIteratedUpdateResult {
  std::size_t iterations{0};
  bool converged{false};
  std::size_t correspondence_count{0};
  SpatialVector pose_increment{SpatialVector::Zero()};
  SpatialVector information_eigenvalues{SpatialVector::Zero()};
  double robust_cost{0.0};
};

struct ForwardSpeedUpdateResult {
  double predicted_speed_mps{0.0};
  double innovation_mps{0.0};
  double innovation_variance{0.0};
};

struct PlanarVelocityUpdateResult {
  Eigen::Vector2d predicted_velocity_mps{Eigen::Vector2d::Zero()};
  Eigen::Vector2d innovation_mps{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d innovation_covariance_mps2{Eigen::Matrix2d::Zero()};
};

struct VerticalPositionUpdateResult {
  double predicted_position_m{0.0};
  double innovation_m{0.0};
  double innovation_variance_m2{0.0};
  double gain{0.0};
};

using MeasurementBuilder =
    std::function<PlanarNormalEquations(const InertialState &)>;
using SpatialMeasurementBuilder =
    std::function<SpatialNormalEquations(const InertialState &)>;

class PlanarIeskf {
public:
  explicit PlanarIeskf(const FilterState &initial_state,
                       const ImuNoise &noise = ImuNoise{},
                       const Eigen::Vector3d &gravity_world =
                           Eigen::Vector3d(0.0, 0.0, -9.80665));

  void reset(const FilterState &state);

  void propagate(const ImuSample &previous, const ImuSample &current);

  IteratedUpdateResult
  update_planar(const MeasurementBuilder &measurement_builder,
                const IteratedUpdateConfig &config = IteratedUpdateConfig{});

  SpatialIteratedUpdateResult
  update_spatial(const SpatialMeasurementBuilder &measurement_builder,
                 const IteratedUpdateConfig &config = IteratedUpdateConfig{});

  // Scalar wheel-speed observation in the vehicle +X direction. It carries no
  // absolute position or heading and is valid during healthy tracking.
  ForwardSpeedUpdateResult update_forward_speed(double speed_mps,
                                                double variance_mps2);

  // Body-frame vx/vy from the vehicle wheel estimator. These high-rate values
  // are correlated, so this anchors planar velocity and its covariance instead
  // of repeatedly treating samples as independent Kalman observations. The
  // transform uses yaw only, matching race_common's planar dead reckoning.
  PlanarVelocityUpdateResult
  update_body_planar_velocity(const Eigen::Vector2d &velocity_body_mps,
                              const Eigen::Vector2d &variance_mps2);

  // Planar-mode fallback only: pin the nuisance Z state to a static map-height
  // profile without modifying x/y/yaw. The full 6DoF localizer rejects this
  // observation because its map update estimates Z directly.
  void pin_vertical_position(double position_z_m, double position_variance_m2,
                             double velocity_variance_mps2);

  // Soft scalar Z observation. It intentionally updates only position.z and
  // never injects TTL information into x/y/yaw or zeros the grade-aware
  // vertical velocity supplied by the IMU + wheel mechanization.
  VerticalPositionUpdateResult
  update_vertical_position(double position_z_m, double position_variance_m2);

  // Uses the live IMU/INS attitude when the selected IMU supplies one. Raw IMU
  // sources omit this observation and retain gyro propagation.
  void pin_orientation(const Eigen::Quaterniond &orientation_world_from_body,
                       const Eigen::Vector3d &variance_rad2);

  const FilterState &state() const noexcept;

private:
  FilterState state_;
  ImuNoise noise_;
  Eigen::Vector3d gravity_world_;
};

} // namespace gicp_localizer::detail
