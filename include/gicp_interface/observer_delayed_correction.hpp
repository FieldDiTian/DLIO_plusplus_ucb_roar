#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace gicp_localizer {

struct ObserverPoseSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = 0.0;
  Eigen::Vector3f p = Eigen::Vector3f::Zero();
  Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
  uint64_t epoch = 0;
};

using ObserverPoseHistory = std::deque<
    ObserverPoseSample, Eigen::aligned_allocator<ObserverPoseSample>>;

inline bool interpolateObserverPose(
    const ObserverPoseHistory& history, double stamp, double max_bracket_gap_s,
    ObserverPoseSample* output) {
  if (!output || history.empty() || !std::isfinite(stamp) ||
      !std::isfinite(max_bracket_gap_s) || max_bracket_gap_s < 0.0) {
    return false;
  }

  const auto upper = std::lower_bound(
      history.begin(), history.end(), stamp,
      [](const ObserverPoseSample& sample, double value) {
        return sample.stamp < value;
      });
  if (upper == history.end()) {
    const auto& nearest = history.back();
    if (std::abs(nearest.stamp - stamp) > max_bracket_gap_s) return false;
    *output = nearest;
    return true;
  }
  if (upper == history.begin()) {
    if (std::abs(upper->stamp - stamp) > max_bracket_gap_s) return false;
    *output = *upper;
    return true;
  }

  const auto lower = std::prev(upper);
  if (lower->epoch != upper->epoch) return false;
  const double lower_dt = stamp - lower->stamp;
  const double upper_dt = upper->stamp - stamp;
  const double span = upper->stamp - lower->stamp;
  if (lower_dt < 0.0 || upper_dt < 0.0 || span <= 0.0 ||
      lower_dt > max_bracket_gap_s || upper_dt > max_bracket_gap_s) {
    return false;
  }

  const float alpha = static_cast<float>(lower_dt / span);
  output->stamp = stamp;
  output->p = (1.0F - alpha) * lower->p + alpha * upper->p;
  output->q = lower->q.slerp(alpha, upper->q).normalized();
  output->epoch = lower->epoch;
  return output->p.allFinite() && output->q.coeffs().allFinite();
}

inline bool observerEpochCompatible(
    const ObserverPoseSample& history_at_measurement,
    const ObserverPoseSample& newest, uint64_t current_epoch,
    uint64_t registration_epoch) {
  return history_at_measurement.epoch == newest.epoch &&
         newest.epoch == current_epoch && current_epoch == registration_epoch;
}

// Build the correction target for the live observer from two poses at the
// SAME historical timestamp.  Applying measurement * inverse(history) to the
// current state preserves all motion that happened after the delayed scan.
inline bool delayedObserverCorrectionTarget(
    const ObserverPoseSample& history_at_measurement,
    const Eigen::Vector3f& measurement_p,
    const Eigen::Quaternionf& measurement_q,
    const Eigen::Vector3f& current_p,
    const Eigen::Quaternionf& current_q,
    Eigen::Vector3f* target_p,
    Eigen::Quaternionf* target_q,
    double* correction_translation_m = nullptr,
    double* correction_rotation_deg = nullptr) {
  if (!target_p || !target_q || !history_at_measurement.p.allFinite() ||
      !history_at_measurement.q.coeffs().allFinite() ||
      !measurement_p.allFinite() || !measurement_q.coeffs().allFinite() ||
      !current_p.allFinite() || !current_q.coeffs().allFinite()) {
    return false;
  }

  const Eigen::Quaternionf q_hist = history_at_measurement.q.normalized();
  const Eigen::Quaternionf q_meas = measurement_q.normalized();
  const Eigen::Quaternionf q_now = current_q.normalized();
  const Eigen::Quaternionf q_corr = (q_meas * q_hist.conjugate()).normalized();
  const Eigen::Vector3f t_corr =
      measurement_p - q_corr._transformVector(history_at_measurement.p);
  *target_p = q_corr._transformVector(current_p) + t_corr;
  *target_q = (q_corr * q_now).normalized();

  if (correction_translation_m) {
    *correction_translation_m =
        static_cast<double>((*target_p - current_p).norm());
  }
  if (correction_rotation_deg) {
    const double w = std::clamp(
        std::abs(static_cast<double>(q_corr.w())), 0.0, 1.0);
    *correction_rotation_deg = 2.0 * std::acos(w) * 180.0 / M_PI;
  }
  return target_p->allFinite() && target_q->coeffs().allFinite();
}

// A public pose may use the observer only to bridge the bounded sensor/compute
// delay after an accepted scan. It must never turn into unbounded IMU-only dead
// reckoning when LiDAR registration stops producing measurements.
inline bool observerDelayWithinBound(double measurement_stamp,
                                     double observer_stamp,
                                     double max_delay_s,
                                     double* delay_s = nullptr) {
  if (!std::isfinite(measurement_stamp) || !std::isfinite(observer_stamp) ||
      !std::isfinite(max_delay_s) || max_delay_s <= 0.0) {
    return false;
  }

  const double delay = observer_stamp - measurement_stamp;
  if (delay_s) *delay_s = delay;
  return delay >= 0.0 && delay <= max_delay_s;
}

struct WheelMotionIntegral {
  bool valid = false;
  double distance_m = 0.0;
  double start_age_s = 0.0;
  double end_age_s = 0.0;
};

// Integrate the non-negative wheel speed over [start_stamp, end_stamp] using a
// zero-order hold. A sample at or before the start anchors the interval; the
// last sample may be held to the end only inside max_endpoint_age_s. This
// deliberately fails closed across wheel dropouts instead of manufacturing a
// plausible-looking displacement from stale speed.
inline WheelMotionIntegral integrateWheelSpeedDistance(
    const std::deque<std::pair<double, double>>& history,
    double start_stamp, double end_stamp, double max_endpoint_age_s) {
  WheelMotionIntegral result;
  if (history.empty() || !std::isfinite(start_stamp) ||
      !std::isfinite(end_stamp) || end_stamp < start_stamp ||
      !std::isfinite(max_endpoint_age_s) || max_endpoint_age_s < 0.0) {
    return result;
  }

  auto upper = std::upper_bound(
      history.begin(), history.end(), start_stamp,
      [](double stamp, const std::pair<double, double>& sample) {
        return stamp < sample.first;
      });
  if (upper == history.begin()) return result;
  auto current = std::prev(upper);
  if (!std::isfinite(current->first) || !std::isfinite(current->second) ||
      current->second < 0.0) {
    return result;
  }
  result.start_age_s = start_stamp - current->first;
  if (result.start_age_s < 0.0 ||
      result.start_age_s > max_endpoint_age_s) {
    return result;
  }

  double cursor = start_stamp;
  double speed = current->second;
  for (auto it = upper; it != history.end() && it->first <= end_stamp; ++it) {
    if (!std::isfinite(it->first) || !std::isfinite(it->second) ||
        it->second < 0.0 || it->first < cursor) {
      return WheelMotionIntegral{};
    }
    result.distance_m += speed * (it->first - cursor);
    cursor = it->first;
    speed = it->second;
  }
  result.end_age_s = end_stamp - cursor;
  if (result.end_age_s < 0.0 || result.end_age_s > max_endpoint_age_s) {
    return WheelMotionIntegral{};
  }
  result.distance_m += speed * result.end_age_s;
  result.valid = std::isfinite(result.distance_m) && result.distance_m >= 0.0;
  return result;
}

// Advance a scan-time pose over a short ground-vehicle latency interval using
// wheel distance for translation magnitude and the observer's relative yaw for
// curvature.  IMU double integration is intentionally not used for XY here:
// an accelerometer bias that is harmless for deskew can create a metre-scale
// speed error when integrated twice.  The observer output still supplies the
// output orientation and Z; absolute odometry is not involved.
inline bool wheelAnchoredPlanarCompensationTarget(
    const Eigen::Vector3f& measurement_p,
    const Eigen::Quaternionf& measurement_q,
    const Eigen::Vector3f& observer_output_p,
    const Eigen::Quaternionf& observer_output_q,
    const WheelMotionIntegral& wheel, Eigen::Vector3f* target_p) {
  if (!target_p || !measurement_p.allFinite() ||
      !measurement_q.coeffs().allFinite() || !observer_output_p.allFinite() ||
      !observer_output_q.coeffs().allFinite() ||
      measurement_q.norm() <= 0.0F || observer_output_q.norm() <= 0.0F ||
      !wheel.valid || !std::isfinite(wheel.distance_m) ||
      wheel.distance_m < 0.0) {
    return false;
  }

  const Eigen::Quaternionf q_measurement = measurement_q.normalized();
  const Eigen::Quaternionf q_output = observer_output_q.normalized();
  const Eigen::Matrix3f relative_rotation =
      (q_measurement.conjugate() * q_output).toRotationMatrix();
  const double yaw_delta = std::atan2(
      static_cast<double>(relative_rotation(1, 0)),
      static_cast<double>(relative_rotation(0, 0)));
  const Eigen::Matrix3f measurement_rotation =
      q_measurement.toRotationMatrix();
  const double measurement_yaw = std::atan2(
      static_cast<double>(measurement_rotation(1, 0)),
      static_cast<double>(measurement_rotation(0, 0)));
  if (!std::isfinite(yaw_delta) || !std::isfinite(measurement_yaw)) {
    return false;
  }

  // Constant-curvature SE(2) integration. wheel.distance_m is arc length.
  // The small-angle branch avoids cancellation and becomes straight motion.
  double delta_body_x = wheel.distance_m;
  double delta_body_y = 0.0;
  if (std::abs(yaw_delta) > 1e-6) {
    delta_body_x = wheel.distance_m * std::sin(yaw_delta) / yaw_delta;
    delta_body_y =
        wheel.distance_m * (1.0 - std::cos(yaw_delta)) / yaw_delta;
  }

  const double cos_yaw = std::cos(measurement_yaw);
  const double sin_yaw = std::sin(measurement_yaw);
  *target_p = observer_output_p;  // preserve the existing bounded Z estimate
  target_p->x() = static_cast<float>(
      static_cast<double>(measurement_p.x()) +
      cos_yaw * delta_body_x - sin_yaw * delta_body_y);
  target_p->y() = static_cast<float>(
      static_cast<double>(measurement_p.y()) +
      sin_yaw * delta_body_x + cos_yaw * delta_body_y);
  return target_p->allFinite();
}

enum class CompensationMotionGateReason {
  kAccepted,
  kInvalidInput,
  kWheelUnavailable,
  kDistanceMismatch,
  kReverseMotion,
  kLateralMotion,
  kYawMotion,
};

inline const char* compensationMotionGateReasonName(
    CompensationMotionGateReason reason) {
  switch (reason) {
    case CompensationMotionGateReason::kAccepted:
      return "compensated";
    case CompensationMotionGateReason::kInvalidInput:
      return "invalid_motion_input";
    case CompensationMotionGateReason::kWheelUnavailable:
      return "wheel_unavailable";
    case CompensationMotionGateReason::kDistanceMismatch:
      return "wheel_distance_mismatch";
    case CompensationMotionGateReason::kReverseMotion:
      return "unexpected_reverse_motion";
    case CompensationMotionGateReason::kLateralMotion:
      return "excess_lateral_motion";
    case CompensationMotionGateReason::kYawMotion:
      return "excess_yaw_motion";
  }
  return "unknown";
}

struct CompensationMotionGateResult {
  CompensationMotionGateReason reason =
      CompensationMotionGateReason::kInvalidInput;
  Eigen::Vector3f delta_body = Eigen::Vector3f::Zero();
  double actual_distance_m = 0.0;
  double expected_distance_m = 0.0;
  double yaw_delta_rad = 0.0;

  bool accepted() const {
    return reason == CompensationMotionGateReason::kAccepted;
  }
};

// Validate only the short motion used to advance an accepted GICP pose. Wheel
// speed supplies path length without absolute-position feedback. The yaw-aware
// lateral allowance admits a legitimate curved chord while rejecting observer
// resets and recovery jumps, which appear as impossible reverse/lateral motion.
inline CompensationMotionGateResult evaluateCompensationMotion(
    const Eigen::Vector3f& measurement_p,
    const Eigen::Quaternionf& measurement_q,
    const Eigen::Vector3f& compensated_p,
    const Eigen::Quaternionf& compensated_q, double delay_s,
    const WheelMotionIntegral& wheel, double distance_abs_tolerance_m,
    double distance_rel_tolerance, double reverse_tolerance_m,
    double lateral_abs_tolerance_m, double lateral_rel_tolerance,
    double yaw_abs_tolerance_rad, double max_yaw_rate_rad_s) {
  CompensationMotionGateResult result;
  if (!measurement_p.allFinite() || !measurement_q.coeffs().allFinite() ||
      !compensated_p.allFinite() || !compensated_q.coeffs().allFinite() ||
      measurement_q.norm() <= 0.0F || compensated_q.norm() <= 0.0F ||
      !std::isfinite(delay_s) || delay_s < 0.0 ||
      !std::isfinite(distance_abs_tolerance_m) ||
      !std::isfinite(distance_rel_tolerance) ||
      !std::isfinite(reverse_tolerance_m) ||
      !std::isfinite(lateral_abs_tolerance_m) ||
      !std::isfinite(lateral_rel_tolerance) ||
      !std::isfinite(yaw_abs_tolerance_rad) ||
      !std::isfinite(max_yaw_rate_rad_s)) {
    return result;
  }
  if (!wheel.valid) {
    result.reason = CompensationMotionGateReason::kWheelUnavailable;
    return result;
  }

  const Eigen::Quaternionf q_measurement = measurement_q.normalized();
  result.delta_body =
      q_measurement.conjugate() * (compensated_p - measurement_p);
  result.actual_distance_m =
      static_cast<double>(result.delta_body.head<2>().norm());
  result.expected_distance_m = wheel.distance_m;
  result.yaw_delta_rad = static_cast<double>(
      q_measurement.angularDistance(compensated_q.normalized()));
  if (!result.delta_body.allFinite() ||
      !std::isfinite(result.actual_distance_m) ||
      !std::isfinite(result.expected_distance_m) ||
      !std::isfinite(result.yaw_delta_rad)) {
    return result;
  }

  const double distance_tolerance =
      std::max(0.0, distance_abs_tolerance_m) +
      std::max(0.0, distance_rel_tolerance) * result.expected_distance_m;
  if (std::abs(result.actual_distance_m - result.expected_distance_m) >
      distance_tolerance) {
    result.reason = CompensationMotionGateReason::kDistanceMismatch;
    return result;
  }
  if (result.expected_distance_m > distance_abs_tolerance_m &&
      static_cast<double>(result.delta_body.x()) <
          -std::max(0.0, reverse_tolerance_m)) {
    result.reason = CompensationMotionGateReason::kReverseMotion;
    return result;
  }

  const double curved_chord_allowance =
      result.expected_distance_m *
      std::sin(std::min(result.yaw_delta_rad, M_PI / 2.0));
  const double lateral_tolerance =
      std::max(0.0, lateral_abs_tolerance_m) + curved_chord_allowance +
      std::max(0.0, lateral_rel_tolerance) * result.expected_distance_m;
  if (std::abs(static_cast<double>(result.delta_body.y())) >
      lateral_tolerance) {
    result.reason = CompensationMotionGateReason::kLateralMotion;
    return result;
  }

  const double yaw_tolerance = std::max(0.0, yaw_abs_tolerance_rad) +
                               std::max(0.0, max_yaw_rate_rad_s) * delay_s;
  if (result.yaw_delta_rad > yaw_tolerance) {
    result.reason = CompensationMotionGateReason::kYawMotion;
    return result;
  }
  result.reason = CompensationMotionGateReason::kAccepted;
  return result;
}

// A wheel-confirmed stationary vehicle has no physical motion to bridge after
// the scan. Use the accepted measurement itself at the newer output stamp;
// residual observer velocity must not create a forward or backward offset.
inline bool stationaryCompensationTarget(
    const Eigen::Vector3f& measurement_p,
    const Eigen::Quaternionf& measurement_q,
    Eigen::Vector3f* target_p,
    Eigen::Quaternionf* target_q) {
  if (!target_p || !target_q || !measurement_p.allFinite() ||
      !measurement_q.coeffs().allFinite() || measurement_q.norm() <= 0.0F) {
    return false;
  }
  *target_p = measurement_p;
  *target_q = measurement_q.normalized();
  return true;
}

}  // namespace gicp_localizer
