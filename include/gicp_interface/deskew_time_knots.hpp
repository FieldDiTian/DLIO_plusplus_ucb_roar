#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace gicp_localizer {

struct DeskewTimeKnots {
  std::vector<double> times;
  double measurement_stamp = 0.0;
  size_t measurement_index = 0;

  bool valid() const {
    return !times.empty() && measurement_index < times.size() &&
           std::isfinite(measurement_stamp);
  }
};

// Pre-composed world<-lidar transforms for one compact trajectory segment.
// For a point at alpha in [0, 1], applying
//
//   (R0 + alpha*dR) * p + (t0 + alpha*dt)
//
// is exactly the linear interpolation of the point transformed at both
// endpoints. With the default 2 ms knot spacing the difference from a full
// quaternion slerp is second order in the inter-knot rotation, while avoiding
// a quaternion construction, normalization, slerp and 4x4 matrix product for
// every LiDAR ray.
struct DeskewAffineSegment {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double first_stamp = 0.0;
  double inverse_span = 0.0;
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f delta_rotation = Eigen::Matrix3f::Zero();
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  Eigen::Vector3f delta_translation = Eigen::Vector3f::Zero();
};

struct DeskewAffineTrajectory {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<double> times;
  std::vector<Eigen::Matrix4f,
              Eigen::aligned_allocator<Eigen::Matrix4f>> world_lidar_poses;
  std::vector<DeskewAffineSegment,
              Eigen::aligned_allocator<DeskewAffineSegment>> segments;

  bool valid() const {
    return !times.empty() && times.size() == world_lidar_poses.size() &&
           segments.size() + 1 == times.size();
  }
};

// Build a compact, deterministic trajectory grid while preserving the exact
// median point timestamp used by the scan-time localization contract. This is
// O(N) for the median plus O(K log K) for K~50 knots, instead of sorting every
// point and producing one IMU pose per unique ray timestamp.
inline DeskewTimeKnots buildDeskewTimeKnots(
    const std::vector<double>& point_times, double interval_s) {
  DeskewTimeKnots result;
  if (point_times.empty() || !std::isfinite(interval_s) || interval_s <= 0.0 ||
      !std::all_of(point_times.begin(), point_times.end(),
                   [](double t) { return std::isfinite(t); })) {
    return result;
  }

  const auto minmax =
      std::minmax_element(point_times.begin(), point_times.end());
  const double first = *minmax.first;
  const double last = *minmax.second;
  if (last < first) return result;

  std::vector<double> median_work = point_times;
  const size_t median_rank = median_work.size() / 2;
  std::nth_element(median_work.begin(), median_work.begin() + median_rank,
                   median_work.end());
  result.measurement_stamp = median_work[median_rank];

  result.times.push_back(first);
  if (last > first) {
    const size_t interior_count = static_cast<size_t>(
        std::floor((last - first) / interval_s));
    // A normal 100 ms sweep at 2 ms produces ~51 knots. Reject absurd spans or
    // configuration instead of allocating an unbounded trajectory.
    if (interior_count > 10000) return DeskewTimeKnots{};
    result.times.reserve(interior_count + 3);
    for (size_t i = 1; i <= interior_count; ++i) {
      const double stamp = first + static_cast<double>(i) * interval_s;
      if (stamp >= last) break;
      result.times.push_back(stamp);
    }
    result.times.push_back(result.measurement_stamp);
    result.times.push_back(last);
  }
  std::sort(result.times.begin(), result.times.end());
  result.times.erase(
      std::unique(result.times.begin(), result.times.end(),
                  [](double a, double b) { return std::abs(a - b) <= 1e-9; }),
      result.times.end());

  const auto measurement = std::min_element(
      result.times.begin(), result.times.end(), [&](double a, double b) {
        return std::abs(a - result.measurement_stamp) <
               std::abs(b - result.measurement_stamp);
      });
  result.measurement_index =
      static_cast<size_t>(std::distance(result.times.begin(), measurement));
  result.measurement_stamp = *measurement;
  return result;
}

inline DeskewAffineTrajectory buildDeskewAffineTrajectory(
    const std::vector<double>& knot_times,
    const std::vector<Eigen::Matrix4f,
                      Eigen::aligned_allocator<Eigen::Matrix4f>>& knot_poses,
    const Eigen::Matrix4f& base_lidar_transform) {
  DeskewAffineTrajectory result;
  if (knot_times.empty() || knot_times.size() != knot_poses.size() ||
      !base_lidar_transform.allFinite()) {
    return result;
  }
  result.times = knot_times;
  result.world_lidar_poses.reserve(knot_poses.size());
  for (const auto& pose : knot_poses) {
    const Eigen::Matrix4f world_lidar = pose * base_lidar_transform;
    if (!world_lidar.allFinite()) return DeskewAffineTrajectory{};
    result.world_lidar_poses.push_back(world_lidar);
  }
  if (knot_times.size() == 1) return result;

  result.segments.reserve(knot_times.size() - 1);
  for (size_t i = 0; i + 1 < knot_times.size(); ++i) {
    const double span = knot_times[i + 1] - knot_times[i];
    if (!std::isfinite(span) || span <= 0.0) {
      return DeskewAffineTrajectory{};
    }
    DeskewAffineSegment segment;
    segment.first_stamp = knot_times[i];
    segment.inverse_span = 1.0 / span;
    segment.rotation =
        result.world_lidar_poses[i].block<3, 3>(0, 0);
    segment.delta_rotation =
        result.world_lidar_poses[i + 1].block<3, 3>(0, 0) -
        segment.rotation;
    segment.translation =
        result.world_lidar_poses[i].block<3, 1>(0, 3);
    segment.delta_translation =
        result.world_lidar_poses[i + 1].block<3, 1>(0, 3) -
        segment.translation;
    result.segments.push_back(segment);
  }
  return result;
}

inline bool interpolateDeskewPointAffine(
    const DeskewAffineTrajectory& trajectory, double stamp,
    const Eigen::Vector3f& lidar_point, Eigen::Vector3f* output) {
  if (!output || !trajectory.valid() || !std::isfinite(stamp) ||
      !lidar_point.allFinite()) {
    return false;
  }
  if (trajectory.times.size() == 1 || stamp <= trajectory.times.front()) {
    *output = trajectory.world_lidar_poses.front().block<3, 3>(0, 0) *
                  lidar_point +
              trajectory.world_lidar_poses.front().block<3, 1>(0, 3);
    return output->allFinite();
  }
  if (stamp >= trajectory.times.back()) {
    *output = trajectory.world_lidar_poses.back().block<3, 3>(0, 0) *
                  lidar_point +
              trajectory.world_lidar_poses.back().block<3, 1>(0, 3);
    return output->allFinite();
  }
  const auto upper =
      std::upper_bound(trajectory.times.begin(), trajectory.times.end(), stamp);
  const size_t upper_index =
      static_cast<size_t>(std::distance(trajectory.times.begin(), upper));
  const auto& segment = trajectory.segments[upper_index - 1];
  const float alpha = static_cast<float>(
      (stamp - segment.first_stamp) * segment.inverse_span);
  *output = segment.rotation * lidar_point + segment.translation +
            alpha * (segment.delta_rotation * lidar_point +
                     segment.delta_translation);
  return output->allFinite();
}

}  // namespace gicp_localizer
