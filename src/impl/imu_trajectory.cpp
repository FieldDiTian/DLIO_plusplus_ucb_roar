#include "gicp_interface/detail/imu_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gicp_localizer::detail {
namespace {

constexpr double kStampToleranceS = 1e-9;

bool finite_sample(const ImuSample &sample) {
  return std::isfinite(sample.stamp_s) && sample.angular_velocity.allFinite() &&
         sample.linear_acceleration.allFinite() &&
         (!sample.orientation_world_from_body.has_value() ||
          (sample.orientation_world_from_body->coeffs().allFinite() &&
           sample.orientation_world_from_body->norm() > 1e-12 &&
           sample.orientation_variance_rad2.has_value() &&
           sample.orientation_variance_rad2->allFinite() &&
           (sample.orientation_variance_rad2->array() > 0.0).all()));
}

bool finite_state(const InertialState &state) {
  return std::isfinite(state.stamp_s) && state.position.allFinite() &&
         state.velocity.allFinite() && state.orientation.coeffs().allFinite() &&
         state.gyro_bias.allFinite() && state.accel_bias.allFinite() &&
         state.orientation.norm() > 1e-12;
}

Eigen::Quaterniond rotation_exp(const Eigen::Vector3d &delta_angle) {
  const double angle = delta_angle.norm();
  if (angle < 1e-12) {
    Eigen::Quaterniond result(1.0, 0.5 * delta_angle.x(), 0.5 * delta_angle.y(),
                              0.5 * delta_angle.z());
    return result.normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, delta_angle / angle));
}

Eigen::Isometry3d pose_from_state(const InertialState &state) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = state.position;
  pose.linear() = state.orientation.toRotationMatrix();
  return pose;
}

} // namespace

ImuTrajectory ImuTrajectory::integrate(const InertialState &anchor,
                                       const std::vector<ImuSample> &samples,
                                       const Eigen::Vector3d &gravity_world) {
  if (!finite_state(anchor) || !gravity_world.allFinite()) {
    throw std::invalid_argument("anchor state and gravity must be finite");
  }
  if (samples.empty()) {
    throw std::invalid_argument("IMU trajectory needs at least one sample");
  }
  if (!finite_sample(samples.front()) ||
      std::abs(samples.front().stamp_s - anchor.stamp_s) > kStampToleranceS) {
    throw std::invalid_argument("first IMU sample must match anchor timestamp");
  }

  ImuTrajectory trajectory;
  InertialState current = anchor;
  current.orientation.normalize();
  trajectory.states_.reserve(samples.size());
  trajectory.states_.push_back(current);

  for (std::size_t i = 1; i < samples.size(); ++i) {
    const ImuSample &previous_sample = samples[i - 1];
    const ImuSample &sample = samples[i];
    if (!finite_sample(previous_sample) || !finite_sample(sample) ||
        sample.stamp_s <= previous_sample.stamp_s) {
      throw std::invalid_argument(
          "IMU samples must be finite and strictly increasing");
    }

    const double dt = sample.stamp_s - previous_sample.stamp_s;
    const Eigen::Vector3d omega_mid =
        0.5 * (previous_sample.angular_velocity + sample.angular_velocity) -
        current.gyro_bias;
    const Eigen::Vector3d accel_mid_body =
        0.5 *
            (previous_sample.linear_acceleration + sample.linear_acceleration) -
        current.accel_bias;

    const Eigen::Quaterniond orientation_mid =
        (current.orientation * rotation_exp(omega_mid * (0.5 * dt)))
            .normalized();
    const Eigen::Vector3d accel_world =
        orientation_mid * accel_mid_body + gravity_world;

    InertialState next = current;
    next.stamp_s = sample.stamp_s;
    next.position =
        current.position + current.velocity * dt + 0.5 * accel_world * dt * dt;
    next.velocity = current.velocity + accel_world * dt;
    next.orientation =
        (current.orientation * rotation_exp(omega_mid * dt)).normalized();
    if (sample.orientation_world_from_body.has_value()) {
      next.orientation = sample.orientation_world_from_body->normalized();
    }
    trajectory.states_.push_back(next);
    current = next;
  }
  return trajectory;
}

bool ImuTrajectory::empty() const noexcept { return states_.empty(); }

double ImuTrajectory::start_time_s() const {
  if (empty()) {
    throw std::logic_error("empty trajectory has no start time");
  }
  return states_.front().stamp_s;
}

double ImuTrajectory::end_time_s() const {
  if (empty()) {
    throw std::logic_error("empty trajectory has no end time");
  }
  return states_.back().stamp_s;
}

std::uint64_t ImuTrajectory::epoch() const {
  if (empty()) {
    throw std::logic_error("empty trajectory has no epoch");
  }
  return states_.front().epoch;
}

InertialState ImuTrajectory::state_at(double stamp_s) const {
  if (empty() || !std::isfinite(stamp_s) ||
      stamp_s < start_time_s() - kStampToleranceS ||
      stamp_s > end_time_s() + kStampToleranceS) {
    throw std::out_of_range("requested state lies outside IMU trajectory");
  }

  if (stamp_s <= start_time_s()) {
    return states_.front();
  }
  if (stamp_s >= end_time_s()) {
    return states_.back();
  }

  const auto upper =
      std::upper_bound(states_.begin(), states_.end(), stamp_s,
                       [](double stamp, const InertialState &state) {
                         return stamp < state.stamp_s;
                       });
  const InertialState &after = *upper;
  const InertialState &before = *(upper - 1);
  const double alpha =
      (stamp_s - before.stamp_s) / (after.stamp_s - before.stamp_s);

  InertialState result = before;
  result.stamp_s = stamp_s;
  result.position =
      before.position + alpha * (after.position - before.position);
  result.velocity =
      before.velocity + alpha * (after.velocity - before.velocity);
  result.orientation =
      before.orientation.slerp(alpha, after.orientation).normalized();
  result.gyro_bias =
      before.gyro_bias + alpha * (after.gyro_bias - before.gyro_bias);
  result.accel_bias =
      before.accel_bias + alpha * (after.accel_bias - before.accel_bias);
  return result;
}

Eigen::Vector3d
ImuTrajectory::deskew_point(const Eigen::Vector3d &point_at_acquisition,
                            double acquisition_time_s,
                            double reference_time_s) const {
  if (!point_at_acquisition.allFinite()) {
    throw std::invalid_argument("point must be finite");
  }
  const Eigen::Isometry3d acquisition_pose =
      pose_from_state(state_at(acquisition_time_s));
  const Eigen::Isometry3d reference_pose =
      pose_from_state(state_at(reference_time_s));
  return reference_pose.inverse() * acquisition_pose * point_at_acquisition;
}

const std::vector<InertialState> &ImuTrajectory::states() const noexcept {
  return states_;
}

} // namespace gicp_localizer::detail
