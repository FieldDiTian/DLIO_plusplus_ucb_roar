#ifndef GICP_LOCALIZER_SCAN_VELOCITY_FEEDBACK_HPP
#define GICP_LOCALIZER_SCAN_VELOCITY_FEEDBACK_HPP

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace gicp_localizer {

// Hysteretic wheel-speed gate for a true zero-motion update. A stale or
// invalid wheel sample must never freeze a moving vehicle, so it fails open.
// The wider exit threshold avoids repeatedly entering/leaving the hold on
// encoder noise around zero.
inline bool updateStationaryMotionGate(bool was_active, bool measurement_fresh,
                                       double measured_speed,
                                       double enter_speed, double exit_speed) {
  if (!measurement_fresh || !std::isfinite(measured_speed) ||
      measured_speed < 0.0 || !std::isfinite(enter_speed) ||
      !std::isfinite(exit_speed) || enter_speed < 0.0 ||
      exit_speed < enter_speed) {
    return false;
  }
  return was_active ? measured_speed < exit_speed
                    : measured_speed <= enter_speed;
}

// Front-only map registration cannot reliably observe motion along the
// vehicle's longitudinal axis on wall/guardrail-dominated track sections.
// Feeding that component back as residual/dt turns a harmless along-track
// pose correction into a large speed error. Keep only the body-lateral and
// vertical components that scan matching can constrain.
inline Eigen::Vector3f
observablePositionResidual(const Eigen::Vector3f &position_residual,
                           const Eigen::Vector3f &body_forward_world) {
  if (!position_residual.allFinite() || !body_forward_world.allFinite()) {
    return Eigen::Vector3f::Zero();
  }
  const float forward_norm = body_forward_world.norm();
  if (!std::isfinite(forward_norm) || forward_norm <= 1e-6F) {
    return Eigen::Vector3f::Zero();
  }
  const Eigen::Vector3f forward = body_forward_world / forward_norm;
  return position_residual - forward * forward.dot(position_residual);
}

// Keep the next scan's integration seed on the LiDAR timeline. The predicted
// velocity and pose residual both describe the current scan reference time;
// no wall-time observer state enters this update.
inline Eigen::Vector3f
scanVelocityFeedback(const Eigen::Vector3f &predicted_velocity,
                     const Eigen::Vector3f &position_residual, double scan_dt,
                     double feedback_gain, double max_speed) {
  Eigen::Vector3f velocity = predicted_velocity.allFinite()
                                 ? predicted_velocity
                                 : Eigen::Vector3f::Zero();
  if (position_residual.allFinite() && std::isfinite(scan_dt) &&
      scan_dt > 1e-3 && std::isfinite(feedback_gain) && feedback_gain > 0.0) {
    velocity += static_cast<float>(feedback_gain / scan_dt) * position_residual;
  }
  if (std::isfinite(max_speed) && max_speed > 0.0) {
    const float speed = velocity.norm();
    if (std::isfinite(speed) && speed > static_cast<float>(max_speed)) {
      velocity *= static_cast<float>(max_speed) / speed;
    }
  }
  return velocity;
}

// Blend a wheel-derived scalar speed into the world-frame body-forward
// velocity component. Lateral/vertical components remain LiDAR/IMU-owned.
inline Eigen::Vector3f
blendForwardSpeed(const Eigen::Vector3f &velocity_world,
                  const Eigen::Vector3f &body_forward_world,
                  double measured_speed, double gain, double max_speed) {
  Eigen::Vector3f velocity =
      velocity_world.allFinite() ? velocity_world : Eigen::Vector3f::Zero();
  if (body_forward_world.allFinite() && std::isfinite(measured_speed) &&
      measured_speed >= 0.0 && std::isfinite(gain) && gain > 0.0) {
    const float forward_norm = body_forward_world.norm();
    if (std::isfinite(forward_norm) && forward_norm > 1e-6F) {
      const Eigen::Vector3f forward = body_forward_world / forward_norm;
      const double bounded_gain = std::clamp(gain, 0.0, 1.0);
      velocity +=
          static_cast<float>(
              bounded_gain *
              (measured_speed - static_cast<double>(forward.dot(velocity)))) *
          forward;
    }
  }
  if (std::isfinite(max_speed) && max_speed > 0.0) {
    const float speed = velocity.norm();
    if (std::isfinite(speed) && speed > static_cast<float>(max_speed)) {
      velocity *= static_cast<float>(max_speed) / speed;
    }
  }
  return velocity;
}

} // namespace gicp_localizer

#endif // GICP_LOCALIZER_SCAN_VELOCITY_FEEDBACK_HPP
