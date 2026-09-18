#include "gicp_interface/detail/localizer_utils.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

constexpr double kOrientationSigmaFloorRad =
    0.25 * 3.14159265358979323846 / 180.0;
constexpr double kWheelUpdatePeriodS = 0.05;
constexpr double kWheelMaximumAgeS = 0.03;
constexpr double kWheelMaximumInnovationMps = 5.0;

gicp_localizer::detail::FilterState makeFusionSeed(
    double stamp_s, const Eigen::Vector3f &position,
    const Eigen::Vector3f &velocity, const Eigen::Quaternionf &orientation,
    const Eigen::Vector3f &gyro_bias, const Eigen::Vector3f &accel_bias,
    uint64_t epoch, const gicp_localizer::detail::ImuSample &sample) {
  gicp_localizer::detail::FilterState seed;
  seed.nominal.stamp_s = stamp_s;
  seed.nominal.position = position.cast<double>();
  seed.nominal.velocity = velocity.cast<double>();
  seed.nominal.orientation = orientation.cast<double>().normalized();
  if (sample.orientation_world_from_body.has_value()) {
    const Eigen::Quaterniond anchored_orientation =
        sample.orientation_world_from_body->normalized();
    seed.nominal.velocity =
        gicp_localizer::reexpressVelocityPreservingBodyComponents(
            seed.nominal.orientation, anchored_orientation,
            seed.nominal.velocity);
    seed.nominal.orientation = anchored_orientation;
  }
  seed.nominal.gyro_bias = gyro_bias.cast<double>();
  seed.nominal.accel_bias = accel_bias.cast<double>();
  seed.nominal.epoch = epoch;

  seed.covariance.setZero();
  seed.covariance.block<3, 3>(0, 0) =
      Eigen::Vector3d(0.25 * 0.25, 0.25 * 0.25, 0.20 * 0.20).asDiagonal();
  seed.covariance.block<3, 3>(3, 3) =
      Eigen::Vector3d::Constant(1.0).asDiagonal();
  Eigen::Vector3d orientation_variance = Eigen::Vector3d::Constant(
      kOrientationSigmaFloorRad * kOrientationSigmaFloorRad);
  if (sample.orientation_variance_rad2.has_value()) {
    orientation_variance = sample.orientation_variance_rad2->cwiseMax(
        kOrientationSigmaFloorRad * kOrientationSigmaFloorRad);
  }
  seed.covariance.block<3, 3>(6, 6) = orientation_variance.asDiagonal();
  seed.covariance.block<3, 3>(9, 9) =
      Eigen::Vector3d::Constant(0.01 * 0.01).asDiagonal();
  seed.covariance.block<3, 3>(12, 12) =
      Eigen::Vector3d::Constant(0.10 * 0.10).asDiagonal();
  return seed;
}

} // namespace

void gicp_localizer::GicpLocalizer::updateDelayedFusionWithImu(
    const gicp_localizer::detail::ImuSample &sample) {
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Quaternionf orientation;
  Eigen::Vector3f gyro_bias;
  Eigen::Vector3f accel_bias;
  uint64_t observer_epoch = 0;
  bool estimator_ready = false;
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    estimator_ready =
        this->initialized.load() && this->geo.first_opt_done.load();
    if (estimator_ready) {
      position = this->state.p;
      velocity = this->state.v.lin.w;
      orientation = this->state.q.normalized();
      gyro_bias = this->state.b.gyro;
      accel_bias = this->state.b.accel;
      observer_epoch = this->observer_epoch_;
    }
  }
  if (!estimator_ready) {
    return;
  }

  std::lock_guard<std::mutex> fusion_lock(this->delayed_fusion_mtx_);
  const auto reset_at_current_sample = [&]() {
    const auto seed =
        makeFusionSeed(sample.stamp_s, position, velocity, orientation,
                       gyro_bias, accel_bias, observer_epoch, sample);
    this->delayed_fusion_.reset(seed, sample);
    this->delayed_fusion_observer_epoch_ = observer_epoch;
    this->delayed_fusion_last_wheel_stamp_s_ = -1.0;
    ++this->delayed_fusion_resets_;
  };

  try {
    if (!this->delayed_fusion_.initialized() ||
        this->delayed_fusion_observer_epoch_ != observer_epoch) {
      reset_at_current_sample();
    } else if (sample.stamp_s > this->delayed_fusion_.newest_time_s()) {
      this->delayed_fusion_.push_imu(sample);
    } else {
      return;
    }
  } catch (const std::exception &error) {
    ++this->delayed_fusion_failures_;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Delayed fusion IMU history reset at %.6f: %s",
                         sample.stamp_s, error.what());
    reset_at_current_sample();
  }

  const auto sync_live_state = [&]() {
    const auto &latest = this->delayed_fusion_.state().nominal;
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    const Eigen::Vector3f angular_velocity_body = this->state.v.ang.b;
    this->state.p = latest.position.cast<float>();
    this->state.q = latest.orientation.cast<float>().normalized();
    this->state.v.lin.w = latest.velocity.cast<float>();
    if (this->gicp_dof_mode_ == "planar" &&
        this->track_z_hold_valid_.load(std::memory_order_acquire)) {
      this->state.p.z() = static_cast<float>(
          this->track_z_hold_m_.load(std::memory_order_relaxed));
      this->state.v.lin.w.z() = 0.0F;
    }
    this->state.v.lin.b = this->state.q.conjugate() * this->state.v.lin.w;
    this->state.v.ang.b = angular_velocity_body;
    this->state.v.ang.w =
        this->state.q.toRotationMatrix() * angular_velocity_body;
    this->state.b.gyro = latest.gyro_bias.cast<float>();
    this->state.b.accel = latest.accel_bias.cast<float>();
    this->geo.prev_p = this->state.p;
    this->geo.prev_q = this->state.q;
    this->geo.prev_vel = this->state.v.lin.w;
    if (!this->observer_pose_history_.empty() &&
        std::abs(this->observer_pose_history_.back().stamp - latest.stamp_s) <
            1e-6) {
      this->observer_pose_history_.back() = {
          latest.stamp_s, this->state.p, this->state.q, this->observer_epoch_};
    }
    ++this->geo.update_seq;
  };

  if (!this->delayed_fusion_latest_wheel_.has_value()) {
    sync_live_state();
    return;
  }
  const FusionWheelSample &wheel = *this->delayed_fusion_latest_wheel_;
  if (wheel.stamp_s <= this->delayed_fusion_last_wheel_stamp_s_ ||
      (this->delayed_fusion_last_wheel_stamp_s_ >= 0.0 &&
       wheel.stamp_s + 1e-6 <
           this->delayed_fusion_last_wheel_stamp_s_ + kWheelUpdatePeriodS)) {
    sync_live_state();
    return;
  }
  const double age_s = this->delayed_fusion_.newest_time_s() - wheel.stamp_s;
  if (age_s < -0.01) {
    sync_live_state();
    return;
  }
  this->delayed_fusion_last_wheel_stamp_s_ = wheel.stamp_s;
  if (age_s > kWheelMaximumAgeS) {
    sync_live_state();
    return;
  }

  const auto &state = this->delayed_fusion_.state().nominal;
  const Eigen::Matrix3d rotation = state.orientation.toRotationMatrix();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const Eigen::Vector2d predicted_velocity(
      cosine * state.velocity.x() + sine * state.velocity.y(),
      -sine * state.velocity.x() + cosine * state.velocity.y());
  if ((wheel.velocity_body_mps - predicted_velocity).cwiseAbs().maxCoeff() >
      kWheelMaximumInnovationMps) {
    sync_live_state();
    return;
  }
  this->delayed_fusion_.apply_body_planar_velocity(wheel.velocity_body_mps,
                                                   wheel.variance_mps2);
  sync_live_state();
}

void gicp_localizer::GicpLocalizer::updateDelayedFusionWithWheel(
    double stamp_s, const Eigen::Vector2d &velocity_body_mps,
    const Eigen::Vector2d &variance_mps2) {
  if (!std::isfinite(stamp_s) || !velocity_body_mps.allFinite() ||
      !variance_mps2.allFinite() || (variance_mps2.array() <= 0.0).any()) {
    return;
  }
  std::lock_guard<std::mutex> fusion_lock(this->delayed_fusion_mtx_);
  if (this->delayed_fusion_latest_wheel_.has_value() &&
      stamp_s <= this->delayed_fusion_latest_wheel_->stamp_s) {
    return;
  }
  this->delayed_fusion_latest_wheel_ =
      FusionWheelSample{stamp_s, velocity_body_mps, variance_mps2};
}

bool gicp_localizer::GicpLocalizer::applyDelayedFusionToAcceptedPose(
    const Eigen::Matrix4f &raw_measurement_pose, bool wheel_stationary_hold,
    DelayedGicpFusionResult *result) {
  if (result == nullptr || !raw_measurement_pose.allFinite() ||
      !std::isfinite(this->t_prior_stamp_)) {
    return false;
  }
  Eigen::Isometry3d measurement = Eigen::Isometry3d::Identity();
  measurement.matrix() = raw_measurement_pose.cast<double>();

  std::lock_guard<std::mutex> fusion_lock(this->delayed_fusion_mtx_);
  if (!this->delayed_fusion_.initialized()) {
    return false;
  }
  try {
    const double filter_time_s = this->delayed_fusion_.newest_time_s();
    // A backlogged stationary scan observes the same vehicle pose at the scan
    // and filter times.  Applying it at the latest filter time is therefore an
    // exact zero-motion time transfer, and avoids a self-sustaining loop where
    // an out-of-history fallback increments the observer epoch on every
    // accepted stationary frame.  Moving scans remain strict delayed updates:
    // we never extend their inertial replay horizon or reinterpret stale poses.
    const double update_time_s =
        wheel_stationary_hold ? filter_time_s : this->t_prior_stamp_;
    *result =
        this->delayed_fusion_.apply_planar_pose(update_time_s, measurement);
    const Eigen::Vector3f raw_position = raw_measurement_pose.block<3, 1>(0, 3);
    Eigen::Quaternionf raw_orientation(raw_measurement_pose.block<3, 3>(0, 0));
    raw_orientation.normalize();

    // Planar GICP/TTL owns the product Z contract.  The delayed filter is used
    // here for x/y/yaw, velocity, covariance and time replay; do not
    // reintroduce an accelerometer-derived height into the existing planar
    // product.
    if (this->gicp_dof_mode_ == "planar") {
      result->measurement_time_state.nominal.position.z() = raw_position.z();
      result->latest_state.nominal.position.z() = raw_position.z();
      result->measurement_time_state.nominal.velocity.z() = 0.0;
      result->latest_state.nominal.velocity.z() = 0.0;
    }

    // Commit the replayed latest state while still holding the fusion lock.
    // IMU callbacks update the legacy observer before acquiring this lock, so
    // this ordering prevents an older post-solve snapshot from overwriting a
    // newer fusion sample.
    {
      std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
      const Eigen::Vector3f angular_velocity_body = this->state.v.ang.b;
      const auto &latest = result->latest_state.nominal;
      this->state.p = latest.position.cast<float>();
      this->state.q = latest.orientation.cast<float>().normalized();
      this->state.v.lin.w = latest.velocity.cast<float>();
      this->state.v.lin.b = this->state.q.conjugate() * this->state.v.lin.w;
      this->state.v.ang.b = angular_velocity_body;
      this->state.v.ang.w =
          this->state.q.toRotationMatrix() * angular_velocity_body;
      this->state.b.gyro = latest.gyro_bias.cast<float>();
      this->state.b.accel = latest.accel_bias.cast<float>();
      this->geo.prev_p = this->state.p;
      this->geo.prev_q = this->state.q;
      this->geo.prev_vel = this->state.v.lin.w;
      this->observer_pose_history_.clear();
      this->observer_pose_history_.push_back({latest.stamp_s, this->state.p,
                                              this->state.q,
                                              this->observer_epoch_});

      auto &output = this->latency_compensated_output_;
      output.valid = true;
      output.stationary = false;
      output.observer_epoch = this->observer_epoch_;
      output.measurement_stamp = this->t_prior_stamp_;
      output.output_stamp = latest.stamp_s;
      output.status = wheel_stationary_hold ? "delayed_ieskf_stationary_latest"
                                            : "delayed_ieskf_replay";
      output.measurement_p = raw_position;
      output.measurement_q = raw_orientation;
      output.p = this->state.p;
      output.q = this->state.q;
      output.v_lin_body = this->state.v.lin.b;
      output.v_ang_body = this->state.v.ang.b;
      ++this->geo.update_seq;
    }
    ++this->delayed_fusion_updates_;
    return true;
  } catch (const std::exception &error) {
    ++this->delayed_fusion_failures_;
    RCLCPP_WARN(this->get_logger(),
                "Delayed GICP fusion rejected scan-time update at %.6f: %s",
                this->t_prior_stamp_, error.what());
    return false;
  }
}
