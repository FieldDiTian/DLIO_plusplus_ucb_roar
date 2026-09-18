#include "gicp_interface/detail/planar_ieskf.hpp"

#include "gicp_interface/detail/planar_pose.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gicp_localizer::detail {
namespace {

constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kOrientationIndex = 6;
constexpr int kGyroBiasIndex = 9;
constexpr int kAccelBiasIndex = 12;

Eigen::Matrix3d skew(const Eigen::Vector3d &value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(), -value.y(),
      value.x(), 0.0;
  return result;
}

Eigen::Quaterniond rotation_exp(const Eigen::Vector3d &delta_angle) {
  const double angle = delta_angle.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond(1.0, 0.5 * delta_angle.x(), 0.5 * delta_angle.y(),
                              0.5 * delta_angle.z())
        .normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, delta_angle / angle));
}

Eigen::Vector3d rotation_log(const Eigen::Quaterniond &input) {
  Eigen::Quaterniond quaternion = input.normalized();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() = -quaternion.coeffs();
  }
  const double vector_norm = quaternion.vec().norm();
  if (vector_norm < 1e-12) {
    return 2.0 * quaternion.vec();
  }
  const double angle = 2.0 * std::atan2(vector_norm, quaternion.w());
  return angle * quaternion.vec() / vector_norm;
}

bool finite_noise(const ImuNoise &noise) {
  return std::isfinite(noise.gyro_noise_density) &&
         noise.gyro_noise_density >= 0.0 &&
         std::isfinite(noise.accel_noise_density) &&
         noise.accel_noise_density >= 0.0 &&
         std::isfinite(noise.gyro_bias_random_walk) &&
         noise.gyro_bias_random_walk >= 0.0 &&
         std::isfinite(noise.accel_bias_random_walk) &&
         noise.accel_bias_random_walk >= 0.0;
}

void validate_state(const FilterState &state) {
  if (!std::isfinite(state.nominal.stamp_s) ||
      !state.nominal.position.allFinite() ||
      !state.nominal.velocity.allFinite() ||
      !state.nominal.orientation.coeffs().allFinite() ||
      state.nominal.orientation.norm() < 1e-12 ||
      !state.nominal.gyro_bias.allFinite() ||
      !state.nominal.accel_bias.allFinite() || !state.covariance.allFinite() ||
      !state.covariance.isApprox(state.covariance.transpose(), 1e-9)) {
    throw std::invalid_argument(
        "filter state must be finite with symmetric covariance");
  }
  const Eigen::SelfAdjointEigenSolver<ErrorCovariance> solver(state.covariance);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues().minCoeff() <= 0.0) {
    throw std::invalid_argument("filter covariance must be positive definite");
  }
}

Eigen::Matrix<double, 3, kErrorStateDimension> planar_projection() {
  Eigen::Matrix<double, 3, kErrorStateDimension> projection =
      Eigen::Matrix<double, 3, kErrorStateDimension>::Zero();
  projection(0, kPositionIndex) = 1.0;
  projection(1, kPositionIndex + 1) = 1.0;
  projection(2, kOrientationIndex + 2) = 1.0;
  return projection;
}

Eigen::Matrix<double, kSpatialPoseDimension, kErrorStateDimension>
spatial_projection() {
  Eigen::Matrix<double, kSpatialPoseDimension, kErrorStateDimension>
      projection = Eigen::Matrix<double, kSpatialPoseDimension,
                                 kErrorStateDimension>::Zero();
  projection.block<3, 3>(0, kPositionIndex) = Eigen::Matrix3d::Identity();
  projection.block<3, 3>(3, kOrientationIndex) = Eigen::Matrix3d::Identity();
  return projection;
}

ErrorVector state_error(const InertialState &reference,
                        const InertialState &current) {
  ErrorVector error = ErrorVector::Zero();
  error.segment<3>(kPositionIndex) = current.position - reference.position;
  error.segment<3>(kVelocityIndex) = current.velocity - reference.velocity;
  error.segment<3>(kOrientationIndex) =
      rotation_log(current.orientation * reference.orientation.conjugate());
  error.segment<3>(kGyroBiasIndex) = current.gyro_bias - reference.gyro_bias;
  error.segment<3>(kAccelBiasIndex) = current.accel_bias - reference.accel_bias;
  return error;
}

void apply_error(InertialState &state, const ErrorVector &error) {
  state.position += error.segment<3>(kPositionIndex);
  state.velocity += error.segment<3>(kVelocityIndex);
  state.orientation =
      (rotation_exp(error.segment<3>(kOrientationIndex)) * state.orientation)
          .normalized();
  state.gyro_bias += error.segment<3>(kGyroBiasIndex);
  state.accel_bias += error.segment<3>(kAccelBiasIndex);
}

} // namespace

PlanarIeskf::PlanarIeskf(const FilterState &initial_state,
                         const ImuNoise &noise,
                         const Eigen::Vector3d &gravity_world)
    : state_(initial_state), noise_(noise), gravity_world_(gravity_world) {
  if (!finite_noise(noise_) || !gravity_world_.allFinite()) {
    throw std::invalid_argument(
        "IMU noise and gravity must be finite and non-negative");
  }
  reset(initial_state);
}

void PlanarIeskf::reset(const FilterState &state) {
  validate_state(state);
  state_ = state;
  state_.nominal.orientation.normalize();
}

void PlanarIeskf::propagate(const ImuSample &previous,
                            const ImuSample &current) {
  if (!std::isfinite(previous.stamp_s) || !std::isfinite(current.stamp_s) ||
      !previous.angular_velocity.allFinite() ||
      !current.angular_velocity.allFinite() ||
      !previous.linear_acceleration.allFinite() ||
      !current.linear_acceleration.allFinite() ||
      std::abs(previous.stamp_s - state_.nominal.stamp_s) > 1e-6 ||
      current.stamp_s <= previous.stamp_s) {
    throw std::invalid_argument(
        "IMU propagation requires finite contiguous increasing samples");
  }
  const double dt = current.stamp_s - previous.stamp_s;
  if (dt > 0.05) {
    throw std::invalid_argument("IMU propagation gap exceeds 50 ms");
  }

  const Eigen::Vector3d omega_body =
      0.5 * (previous.angular_velocity + current.angular_velocity) -
      state_.nominal.gyro_bias;
  const Eigen::Vector3d accel_body =
      0.5 * (previous.linear_acceleration + current.linear_acceleration) -
      state_.nominal.accel_bias;
  const Eigen::Quaterniond orientation_mid =
      (state_.nominal.orientation * rotation_exp(omega_body * (0.5 * dt)))
          .normalized();
  const Eigen::Matrix3d rotation_mid = orientation_mid.toRotationMatrix();
  const Eigen::Vector3d specific_force_world = rotation_mid * accel_body;
  const Eigen::Vector3d accel_world = specific_force_world + gravity_world_;

  state_.nominal.position +=
      state_.nominal.velocity * dt + 0.5 * accel_world * dt * dt;
  state_.nominal.velocity += accel_world * dt;
  state_.nominal.orientation =
      (state_.nominal.orientation * rotation_exp(omega_body * dt)).normalized();
  state_.nominal.stamp_s = current.stamp_s;

  ErrorCovariance dynamics = ErrorCovariance::Zero();
  dynamics.block<3, 3>(kPositionIndex, kVelocityIndex) =
      Eigen::Matrix3d::Identity();
  dynamics.block<3, 3>(kVelocityIndex, kOrientationIndex) =
      -skew(specific_force_world);
  dynamics.block<3, 3>(kVelocityIndex, kAccelBiasIndex) = -rotation_mid;
  dynamics.block<3, 3>(kOrientationIndex, kGyroBiasIndex) = -rotation_mid;
  const ErrorCovariance transition =
      ErrorCovariance::Identity() + dynamics * dt;

  ErrorCovariance process_noise = ErrorCovariance::Zero();
  process_noise.block<3, 3>(kVelocityIndex, kVelocityIndex) =
      std::pow(noise_.accel_noise_density, 2) * Eigen::Matrix3d::Identity();
  process_noise.block<3, 3>(kOrientationIndex, kOrientationIndex) =
      std::pow(noise_.gyro_noise_density, 2) * Eigen::Matrix3d::Identity();
  process_noise.block<3, 3>(kGyroBiasIndex, kGyroBiasIndex) =
      std::pow(noise_.gyro_bias_random_walk, 2) * Eigen::Matrix3d::Identity();
  process_noise.block<3, 3>(kAccelBiasIndex, kAccelBiasIndex) =
      std::pow(noise_.accel_bias_random_walk, 2) * Eigen::Matrix3d::Identity();
  state_.covariance = transition * state_.covariance * transition.transpose() +
                      process_noise * dt;
  state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
}

IteratedUpdateResult
PlanarIeskf::update_planar(const MeasurementBuilder &measurement_builder,
                           const IteratedUpdateConfig &config) {
  if (!measurement_builder || config.maximum_iterations == 0 ||
      !std::isfinite(config.translation_convergence_m) ||
      !std::isfinite(config.rotation_convergence_rad) ||
      !std::isfinite(config.minimum_information_eigenvalue) ||
      config.translation_convergence_m <= 0.0 ||
      config.rotation_convergence_rad <= 0.0 ||
      config.minimum_information_eigenvalue < 0.0) {
    throw std::invalid_argument("invalid iterated update configuration");
  }

  const FilterState prior = state_;
  const Eigen::LDLT<ErrorCovariance> prior_factorization(prior.covariance);
  if (prior_factorization.info() != Eigen::Success ||
      !prior_factorization.isPositive()) {
    throw std::runtime_error("prior covariance is not positive definite");
  }
  const ErrorCovariance prior_information =
      prior_factorization.solve(ErrorCovariance::Identity());
  const auto projection = planar_projection();

  IteratedUpdateResult result;
  PlanarNormalEquations final_equations;
  for (std::size_t iteration = 0; iteration < config.maximum_iterations;
       ++iteration) {
    const PlanarNormalEquations equations = measurement_builder(state_.nominal);
    if (equations.correspondence_count == 0 || !equations.hessian.allFinite() ||
        !equations.gradient.allFinite()) {
      throw std::runtime_error(
          "planar measurement has no finite correspondences");
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> information_solver(
        equations.hessian);
    if (information_solver.info() != Eigen::Success ||
        information_solver.eigenvalues().minCoeff() <
            config.minimum_information_eigenvalue) {
      throw std::runtime_error("planar measurement is degenerate");
    }

    const ErrorCovariance measurement_information =
        projection.transpose() * equations.hessian * projection;
    const ErrorVector measurement_gradient =
        projection.transpose() * equations.gradient;
    const ErrorVector current_error =
        state_error(prior.nominal, state_.nominal);
    const ErrorCovariance system = prior_information + measurement_information;
    const Eigen::LDLT<ErrorCovariance> system_factorization(system);
    if (system_factorization.info() != Eigen::Success ||
        !system_factorization.isPositive()) {
      throw std::runtime_error(
          "iterated filter system is not positive definite");
    }
    ErrorVector step = system_factorization.solve(
        -measurement_gradient - prior_information * current_error);
    if (!step.allFinite()) {
      throw std::runtime_error("iterated filter produced non-finite step");
    }
    // A planar map observation may transfer information to correlated planar
    // velocity and bias states, but it must never become an implicit Z or
    // roll/pitch measurement through covariance cross terms. Those nuisance
    // states remain owned by the TTL/IMU/wheel observer.
    step(kPositionIndex + 2) = 0.0;
    step(kVelocityIndex + 2) = 0.0;
    step(kOrientationIndex) = 0.0;
    step(kOrientationIndex + 1) = 0.0;
    apply_error(state_.nominal, step);

    ++result.iterations;
    final_equations = equations;
    result.correspondence_count = equations.correspondence_count;
    result.robust_cost = equations.robust_cost;
    result.information_eigenvalues = information_solver.eigenvalues();
    if (step.segment<2>(kPositionIndex).norm() <=
            config.translation_convergence_m &&
        std::abs(step(kOrientationIndex + 2)) <=
            config.rotation_convergence_rad) {
      result.converged = true;
      break;
    }
  }

  const ErrorCovariance posterior_information =
      prior_information +
      projection.transpose() * final_equations.hessian * projection;
  const Eigen::LDLT<ErrorCovariance> posterior_factorization(
      posterior_information);
  if (posterior_factorization.info() != Eigen::Success ||
      !posterior_factorization.isPositive()) {
    throw std::runtime_error("posterior information is not positive definite");
  }
  state_.covariance =
      posterior_factorization.solve(ErrorCovariance::Identity());
  state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());

  const PlanarPose prior_planar = [&prior]() {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = prior.nominal.position;
    pose.linear() = prior.nominal.orientation.toRotationMatrix();
    return extract_planar_pose(pose);
  }();
  Eigen::Isometry3d posterior_pose = Eigen::Isometry3d::Identity();
  posterior_pose.translation() = state_.nominal.position;
  posterior_pose.linear() = state_.nominal.orientation.toRotationMatrix();
  const PlanarPose posterior_planar = extract_planar_pose(posterior_pose);
  result.planar_increment = Eigen::Vector3d(
      posterior_planar.x - prior_planar.x, posterior_planar.y - prior_planar.y,
      normalize_yaw(posterior_planar.yaw - prior_planar.yaw));
  return result;
}

SpatialIteratedUpdateResult PlanarIeskf::update_spatial(
    const SpatialMeasurementBuilder &measurement_builder,
    const IteratedUpdateConfig &config) {
  if (!measurement_builder || config.maximum_iterations == 0 ||
      !std::isfinite(config.translation_convergence_m) ||
      !std::isfinite(config.rotation_convergence_rad) ||
      !std::isfinite(config.minimum_information_eigenvalue) ||
      config.translation_convergence_m <= 0.0 ||
      config.rotation_convergence_rad <= 0.0 ||
      config.minimum_information_eigenvalue < 0.0) {
    throw std::invalid_argument(
        "invalid spatial iterated update configuration");
  }

  const FilterState prior = state_;
  const Eigen::LDLT<ErrorCovariance> prior_factorization(prior.covariance);
  if (prior_factorization.info() != Eigen::Success ||
      !prior_factorization.isPositive()) {
    throw std::runtime_error("prior covariance is not positive definite");
  }
  const ErrorCovariance prior_information =
      prior_factorization.solve(ErrorCovariance::Identity());
  const auto projection = spatial_projection();

  SpatialIteratedUpdateResult result;
  SpatialNormalEquations final_equations;
  for (std::size_t iteration = 0; iteration < config.maximum_iterations;
       ++iteration) {
    const SpatialNormalEquations equations =
        measurement_builder(state_.nominal);
    if (equations.correspondence_count == 0 || !equations.hessian.allFinite() ||
        !equations.gradient.allFinite()) {
      throw std::runtime_error(
          "spatial measurement has no finite correspondences");
    }
    const Eigen::SelfAdjointEigenSolver<SpatialMatrix> information_solver(
        equations.hessian);
    if (information_solver.info() != Eigen::Success ||
        information_solver.eigenvalues().minCoeff() <
            config.minimum_information_eigenvalue) {
      throw std::runtime_error("spatial measurement is degenerate");
    }

    const ErrorCovariance measurement_information =
        projection.transpose() * equations.hessian * projection;
    const ErrorVector measurement_gradient =
        projection.transpose() * equations.gradient;
    const ErrorVector current_error =
        state_error(prior.nominal, state_.nominal);
    const ErrorCovariance system = prior_information + measurement_information;
    const Eigen::LDLT<ErrorCovariance> system_factorization(system);
    if (system_factorization.info() != Eigen::Success ||
        !system_factorization.isPositive()) {
      throw std::runtime_error(
          "spatial iterated filter system is not positive definite");
    }
    const ErrorVector step = system_factorization.solve(
        -measurement_gradient - prior_information * current_error);
    if (!step.allFinite()) {
      throw std::runtime_error(
          "spatial iterated filter produced non-finite step");
    }
    apply_error(state_.nominal, step);

    ++result.iterations;
    final_equations = equations;
    result.correspondence_count = equations.correspondence_count;
    result.robust_cost = equations.robust_cost;
    result.information_eigenvalues = information_solver.eigenvalues();
    if (step.segment<3>(kPositionIndex).norm() <=
            config.translation_convergence_m &&
        step.segment<3>(kOrientationIndex).norm() <=
            config.rotation_convergence_rad) {
      result.converged = true;
      break;
    }
  }

  const ErrorCovariance posterior_information =
      prior_information +
      projection.transpose() * final_equations.hessian * projection;
  const Eigen::LDLT<ErrorCovariance> posterior_factorization(
      posterior_information);
  if (posterior_factorization.info() != Eigen::Success ||
      !posterior_factorization.isPositive()) {
    throw std::runtime_error("posterior information is not positive definite");
  }
  state_.covariance =
      posterior_factorization.solve(ErrorCovariance::Identity());
  state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());

  result.pose_increment.head<3>() =
      state_.nominal.position - prior.nominal.position;
  result.pose_increment.tail<3>() = rotation_log(
      state_.nominal.orientation * prior.nominal.orientation.conjugate());
  return result;
}

ForwardSpeedUpdateResult
PlanarIeskf::update_forward_speed(double speed_mps, double variance_mps2) {
  if (!std::isfinite(speed_mps) || !std::isfinite(variance_mps2) ||
      variance_mps2 <= 0.0) {
    throw std::invalid_argument(
        "forward speed and variance must be finite with positive variance");
  }

  const Eigen::Vector3d forward_world =
      state_.nominal.orientation * Eigen::Vector3d::UnitX();
  Eigen::Matrix<double, 1, kErrorStateDimension> jacobian =
      Eigen::Matrix<double, 1, kErrorStateDimension>::Zero();
  jacobian.segment<3>(kVelocityIndex) = forward_world.transpose();
  // Treat attitude as fixed for this scalar observation. On a race car the
  // velocity vector contains real sideslip; allowing a body-X speed residual
  // to update orientation incorrectly converts sideslip into yaw correction.

  ForwardSpeedUpdateResult result;
  result.predicted_speed_mps = forward_world.dot(state_.nominal.velocity);
  result.innovation_mps = speed_mps - result.predicted_speed_mps;
  result.innovation_variance =
      (jacobian * state_.covariance * jacobian.transpose())(0, 0) +
      variance_mps2;
  if (!std::isfinite(result.innovation_variance) ||
      result.innovation_variance <= 0.0) {
    throw std::runtime_error("forward speed innovation variance is invalid");
  }

  Eigen::Matrix<double, kErrorStateDimension, 1> gain =
      Eigen::Matrix<double, kErrorStateDimension, 1>::Zero();
  gain.segment<3>(kVelocityIndex) =
      state_.covariance.block<3, 3>(kVelocityIndex, kVelocityIndex) *
      forward_world / result.innovation_variance;
  apply_error(state_.nominal, gain * result.innovation_mps);

  const ErrorCovariance identity = ErrorCovariance::Identity();
  const ErrorCovariance residual_projection = identity - gain * jacobian;
  state_.covariance = residual_projection * state_.covariance *
                          residual_projection.transpose() +
                      variance_mps2 * gain * gain.transpose();
  state_.covariance = 0.5 * (state_.covariance + state_.covariance.transpose());
  return result;
}

PlanarVelocityUpdateResult PlanarIeskf::update_body_planar_velocity(
    const Eigen::Vector2d &velocity_body_mps,
    const Eigen::Vector2d &variance_mps2) {
  if (!velocity_body_mps.allFinite() || !variance_mps2.allFinite() ||
      (variance_mps2.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "planar velocity and variances must be finite with positive variance");
  }

  const Eigen::Matrix3d rotation_world_from_body =
      state_.nominal.orientation.toRotationMatrix();
  const double yaw = std::atan2(rotation_world_from_body(1, 0),
                                rotation_world_from_body(0, 0));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Eigen::Matrix<double, 2, 3> body_planar_from_world_velocity;
  body_planar_from_world_velocity << cosine, sine, 0.0, -sine, cosine, 0.0;

  Eigen::Matrix<double, 2, kErrorStateDimension> jacobian =
      Eigen::Matrix<double, 2, kErrorStateDimension>::Zero();
  jacobian.block<2, 3>(0, kVelocityIndex) = body_planar_from_world_velocity;
  const Eigen::Matrix2d measurement_covariance = variance_mps2.asDiagonal();

  PlanarVelocityUpdateResult result;
  result.predicted_velocity_mps =
      body_planar_from_world_velocity * state_.nominal.velocity;
  result.innovation_mps = velocity_body_mps - result.predicted_velocity_mps;
  result.innovation_covariance_mps2 =
      jacobian * state_.covariance * jacobian.transpose() +
      measurement_covariance;
  const Eigen::Matrix2d world_planar_from_body =
      body_planar_from_world_velocity.leftCols<2>().transpose();
  const Eigen::Vector3d full_world_velocity =
      state_.nominal.orientation *
      Eigen::Vector3d(velocity_body_mps.x(), velocity_body_mps.y(), 0.0);
  state_.nominal.velocity.head<2>() =
      world_planar_from_body * velocity_body_mps;
  state_.nominal.velocity.z() = full_world_velocity.z();

  for (int axis = 0; axis < 3; ++axis) {
    state_.covariance.row(kVelocityIndex + axis).setZero();
    state_.covariance.col(kVelocityIndex + axis).setZero();
  }
  const Eigen::Vector3d body_velocity_variance(
      variance_mps2.x(), variance_mps2.y(), variance_mps2.maxCoeff());
  state_.covariance.block<3, 3>(kVelocityIndex, kVelocityIndex) =
      state_.nominal.orientation.toRotationMatrix() *
      body_velocity_variance.asDiagonal() *
      state_.nominal.orientation.toRotationMatrix().transpose();
  return result;
}

void PlanarIeskf::pin_vertical_position(double position_z_m,
                                        double position_variance_m2,
                                        double velocity_variance_mps2) {
  if (!std::isfinite(position_z_m) || !std::isfinite(position_variance_m2) ||
      !std::isfinite(velocity_variance_mps2) || position_variance_m2 <= 0.0 ||
      velocity_variance_mps2 <= 0.0) {
    throw std::invalid_argument(
        "vertical reference must be finite with positive variance");
  }

  constexpr int kPositionZIndex = kPositionIndex + 2;
  constexpr int kVelocityZIndex = kVelocityIndex + 2;
  state_.nominal.position.z() = position_z_m;
  state_.nominal.velocity.z() = 0.0;
  state_.covariance.row(kPositionZIndex).setZero();
  state_.covariance.col(kPositionZIndex).setZero();
  state_.covariance.row(kVelocityZIndex).setZero();
  state_.covariance.col(kVelocityZIndex).setZero();
  state_.covariance(kPositionZIndex, kPositionZIndex) = position_variance_m2;
  state_.covariance(kVelocityZIndex, kVelocityZIndex) = velocity_variance_mps2;
}

VerticalPositionUpdateResult
PlanarIeskf::update_vertical_position(double position_z_m,
                                      double position_variance_m2) {
  if (!std::isfinite(position_z_m) || !std::isfinite(position_variance_m2) ||
      position_variance_m2 <= 0.0) {
    throw std::invalid_argument(
        "vertical observation must be finite with positive variance");
  }

  constexpr int kPositionZIndex = kPositionIndex + 2;
  const double prior_variance_m2 =
      state_.covariance(kPositionZIndex, kPositionZIndex);
  if (!std::isfinite(prior_variance_m2) || prior_variance_m2 <= 0.0) {
    throw std::runtime_error("vertical position variance is invalid");
  }

  VerticalPositionUpdateResult result;
  result.predicted_position_m = state_.nominal.position.z();
  result.innovation_m = position_z_m - result.predicted_position_m;
  result.innovation_variance_m2 =
      prior_variance_m2 + position_variance_m2;
  result.gain = prior_variance_m2 / result.innovation_variance_m2;
  state_.nominal.position.z() += result.gain * result.innovation_m;

  // Z is deliberately isolated from the persistent planar state. Retaining
  // cross-covariance while refusing the corresponding x/y/yaw correction would
  // make the covariance inconsistent, so remove those cross terms and keep the
  // exact scalar posterior variance.
  state_.covariance.row(kPositionZIndex).setZero();
  state_.covariance.col(kPositionZIndex).setZero();
  state_.covariance(kPositionZIndex, kPositionZIndex) =
      (1.0 - result.gain) * prior_variance_m2;
  return result;
}

void PlanarIeskf::pin_orientation(
    const Eigen::Quaterniond &orientation_world_from_body,
    const Eigen::Vector3d &variance_rad2) {
  if (!orientation_world_from_body.coeffs().allFinite() ||
      orientation_world_from_body.norm() < 1e-12 ||
      !variance_rad2.allFinite() || (variance_rad2.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "orientation reference must be finite with positive variance");
  }
  state_.nominal.orientation = orientation_world_from_body.normalized();
  for (int axis = 0; axis < 3; ++axis) {
    state_.covariance.row(kOrientationIndex + axis).setZero();
    state_.covariance.col(kOrientationIndex + axis).setZero();
  }
  state_.covariance.block<3, 3>(kOrientationIndex, kOrientationIndex) =
      variance_rad2.asDiagonal();
}

const FilterState &PlanarIeskf::state() const noexcept { return state_; }

} // namespace gicp_localizer::detail
