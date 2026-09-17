#include "gicp_interface/delayed_gicp_fusion.hpp"

#include "gicp_interface/detail/planar_pose.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <stdexcept>

namespace gicp_localizer {
namespace {

void validate_config(const DelayedGicpFusionConfig &config) {
  if (!std::isfinite(config.history_duration_s) ||
      config.history_duration_s <= 0.0 ||
      !std::isfinite(config.pose_xy_sigma_m) || config.pose_xy_sigma_m <= 0.0 ||
      !std::isfinite(config.pose_yaw_sigma_rad) ||
      config.pose_yaw_sigma_rad <= 0.0 ||
      !std::isfinite(config.minimum_information_eigenvalue) ||
      config.minimum_information_eigenvalue < 0.0) {
    throw std::invalid_argument("invalid delayed GICP fusion configuration");
  }
}

Eigen::Isometry3d pose_from_state(const gicp_localizer::detail::InertialState &state) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = state.position;
  pose.linear() = state.orientation.toRotationMatrix();
  return pose;
}

} // namespace

Eigen::Vector3d reexpressVelocityPreservingBodyComponents(
    const Eigen::Quaterniond &previous_world_from_body,
    const Eigen::Quaterniond &updated_world_from_body,
    const Eigen::Vector3d &previous_velocity_world) {
  if (!previous_world_from_body.coeffs().allFinite() ||
      !updated_world_from_body.coeffs().allFinite() ||
      !previous_velocity_world.allFinite() ||
      previous_world_from_body.norm() <= 1e-12 ||
      updated_world_from_body.norm() <= 1e-12) {
    throw std::invalid_argument(
        "velocity attitude re-expression inputs must be finite");
  }
  const Eigen::Quaterniond previous = previous_world_from_body.normalized();
  const Eigen::Quaterniond updated = updated_world_from_body.normalized();
  const Eigen::Vector3d velocity_body =
      previous.conjugate() * previous_velocity_world;
  return updated * velocity_body;
}

DelayedGicpFusion::DelayedGicpFusion(const DelayedGicpFusionConfig &config)
    : config_(config) {
  validate_config(config_);
}

void DelayedGicpFusion::reset(const gicp_localizer::detail::FilterState &state,
                              const gicp_localizer::detail::ImuSample &sample) {
  filter_ = std::make_unique<gicp_localizer::detail::DelayedIeskf>(
      state, sample, config_.history_duration_s, config_.imu_noise);
}

bool DelayedGicpFusion::initialized() const noexcept {
  return static_cast<bool>(filter_);
}

void DelayedGicpFusion::push_imu(const gicp_localizer::detail::ImuSample &sample) {
  if (!filter_) {
    throw std::logic_error("delayed GICP fusion is not initialized");
  }
  filter_->push_imu(sample);
}

gicp_localizer::detail::PlanarVelocityUpdateResult
DelayedGicpFusion::apply_body_planar_velocity(
    const Eigen::Vector2d &velocity_body_mps,
    const Eigen::Vector2d &variance_mps2) {
  if (!filter_) {
    throw std::logic_error("delayed GICP fusion is not initialized");
  }
  return filter_->apply_latest_body_planar_velocity(velocity_body_mps,
                                                    variance_mps2);
}

DelayedGicpFusionResult DelayedGicpFusion::apply_planar_pose(
    double measurement_time_s,
    const Eigen::Isometry3d &world_from_body_measurement) {
  if (!filter_) {
    throw std::logic_error("delayed GICP fusion is not initialized");
  }
  if (!std::isfinite(measurement_time_s) ||
      !world_from_body_measurement.matrix().allFinite()) {
    throw std::invalid_argument("GICP pose measurement must be finite");
  }

  const gicp_localizer::detail::PlanarPose measurement =
      gicp_localizer::detail::extract_planar_pose(world_from_body_measurement);
  const Eigen::Vector3d information_diagonal(
      1.0 / (config_.pose_xy_sigma_m * config_.pose_xy_sigma_m),
      1.0 / (config_.pose_xy_sigma_m * config_.pose_xy_sigma_m),
      1.0 / (config_.pose_yaw_sigma_rad * config_.pose_yaw_sigma_rad));
  const Eigen::Matrix3d information = information_diagonal.asDiagonal();

  auto measurement_builder =
      [measurement, information](const gicp_localizer::detail::InertialState &state) {
        const gicp_localizer::detail::PlanarPose current =
            gicp_localizer::detail::extract_planar_pose(pose_from_state(state));
        Eigen::Vector3d residual(
            current.x - measurement.x, current.y - measurement.y,
            gicp_localizer::detail::normalize_yaw(current.yaw - measurement.yaw));
        gicp_localizer::detail::PlanarNormalEquations equations;
        equations.hessian = information;
        equations.gradient = information * residual;
        equations.robust_cost = residual.dot(information * residual);
        equations.correspondence_count = 1;
        return equations;
      };

  gicp_localizer::detail::IteratedUpdateConfig update_config;
  update_config.maximum_iterations = 2;
  update_config.translation_convergence_m = 1e-5;
  update_config.rotation_convergence_rad = 1e-6;
  update_config.minimum_information_eigenvalue =
      config_.minimum_information_eigenvalue;

  const gicp_localizer::detail::PlanarPose prior =
      gicp_localizer::detail::extract_planar_pose(
          pose_from_state(filter_->state_at(measurement_time_s).nominal));
  const gicp_localizer::detail::DelayedUpdateResult update =
      filter_->apply_delayed_planar_update(measurement_time_s,
                                           measurement_builder, update_config);

  DelayedGicpFusionResult result;
  result.measurement_time_state = update.measurement_time_state;
  result.latest_state = update.latest_state;
  result.replayed_imu_samples = update.replayed_imu_samples;
  result.delay_s = update.delay_s;
  result.planar_innovation = Eigen::Vector3d(
      measurement.x - prior.x, measurement.y - prior.y,
      gicp_localizer::detail::normalize_yaw(measurement.yaw - prior.yaw));
  return result;
}

const gicp_localizer::detail::FilterState &DelayedGicpFusion::state() const {
  if (!filter_) {
    throw std::logic_error("delayed GICP fusion is not initialized");
  }
  return filter_->state();
}

gicp_localizer::detail::FilterState DelayedGicpFusion::state_at(double stamp_s) const {
  if (!filter_) {
    throw std::logic_error("delayed GICP fusion is not initialized");
  }
  return filter_->state_at(stamp_s);
}

double DelayedGicpFusion::newest_time_s() const {
  if (!filter_) {
    throw std::logic_error("delayed GICP fusion is not initialized");
  }
  return filter_->newest_time_s();
}

double DelayedGicpFusion::oldest_time_s() const {
  if (!filter_) {
    throw std::logic_error("delayed GICP fusion is not initialized");
  }
  return filter_->oldest_time_s();
}

} // namespace gicp_localizer
