#ifndef GICP_LOCALIZER_POINT_BUDGET_HPP
#define GICP_LOCALIZER_POINT_BUDGET_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace gicp_localizer {

// Deterministically spread a fixed-size sample across an ordered point set.
// PCL VoxelGrid emits a deterministic voxel order, so selecting evenly over
// that order preserves broad spatial coverage without a random generator or
// another nearest-neighbor pass.
inline std::vector<size_t> evenlySpacedPointIndices(size_t input_size,
                                                    size_t max_points) {
  if (max_points == 0 || input_size <= max_points) {
    return {};
  }
  if (max_points == 1) {
    return {input_size / 2};
  }

  std::vector<size_t> indices;
  indices.reserve(max_points);
  for (size_t i = 0; i < max_points; ++i) {
    indices.push_back(i * (input_size - 1) / (max_points - 1));
  }
  return indices;
}

// Preserve broad scan geometry when a dense cloud exceeds its point budget.
// Points are grouped by azimuth around the LiDAR and by sqrt-scaled range, so
// nearby structure cannot consume the whole budget while guardrails and other
// sparse long-range returns disappear. Each non-empty cell receives a fair
// share, and selection inside a cell remains deterministic.
template <typename PointT, typename Allocator>
inline std::vector<size_t> spatiallyBalancedPointIndices(
    const std::vector<PointT, Allocator> &points, size_t max_points,
    const std::array<double, 3> &sensor_origin, double max_range_m,
    size_t azimuth_bins, size_t range_bins) {
  if (max_points == 0 || points.size() <= max_points) {
    return {};
  }
  if (azimuth_bins == 0 || range_bins == 0 ||
      !std::isfinite(sensor_origin[0]) || !std::isfinite(sensor_origin[1]) ||
      !std::isfinite(sensor_origin[2]) || !std::isfinite(max_range_m) ||
      max_range_m <= 0.0 ||
      azimuth_bins > std::numeric_limits<size_t>::max() / range_bins) {
    return evenlySpacedPointIndices(points.size(), max_points);
  }

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  const size_t bucket_count = azimuth_bins * range_bins;
  std::vector<std::vector<size_t>> buckets(bucket_count);

  for (size_t index = 0; index < points.size(); ++index) {
    const double dx = static_cast<double>(points[index].x) - sensor_origin[0];
    const double dy = static_cast<double>(points[index].y) - sensor_origin[1];
    const double dz = static_cast<double>(points[index].z) - sensor_origin[2];
    const double range_sq = dx * dx + dy * dy + dz * dz;
    if (!std::isfinite(range_sq)) {
      return evenlySpacedPointIndices(points.size(), max_points);
    }

    const double azimuth_unit =
        std::clamp((std::atan2(dy, dx) + kPi) / kTwoPi, 0.0, 1.0);
    const size_t azimuth_bin = std::min(
        static_cast<size_t>(azimuth_unit * azimuth_bins), azimuth_bins - 1);
    // sqrt scaling gives sparse mid/far returns a useful share without making
    // the innermost range layer dominate a front-LiDAR scan.
    const double range_unit =
        std::sqrt(std::clamp(std::sqrt(range_sq) / max_range_m, 0.0, 1.0));
    const size_t range_bin =
        std::min(static_cast<size_t>(range_unit * range_bins), range_bins - 1);
    buckets[range_bin * azimuth_bins + azimuth_bin].push_back(index);
  }

  std::vector<size_t> quotas(bucket_count, 0);
  size_t remaining = max_points;
  while (remaining > 0) {
    size_t active = 0;
    for (size_t bucket = 0; bucket < bucket_count; ++bucket) {
      active += quotas[bucket] < buckets[bucket].size() ? 1 : 0;
    }
    if (active == 0) {
      break;
    }

    const size_t share = std::max<size_t>(remaining / active, 1);
    for (size_t bucket = 0; bucket < bucket_count && remaining > 0; ++bucket) {
      const size_t available = buckets[bucket].size() - quotas[bucket];
      const size_t take = std::min({available, share, remaining});
      quotas[bucket] += take;
      remaining -= take;
    }
  }

  std::vector<size_t> indices;
  indices.reserve(max_points - remaining);
  for (size_t bucket = 0; bucket < bucket_count; ++bucket) {
    const size_t selected = quotas[bucket];
    const size_t available = buckets[bucket].size();
    if (selected == 0) {
      continue;
    }
    if (selected == 1) {
      indices.push_back(buckets[bucket][available / 2]);
      continue;
    }
    for (size_t i = 0; i < selected; ++i) {
      const size_t position = i * (available - 1) / (selected - 1);
      indices.push_back(buckets[bucket][position]);
    }
  }
  return indices;
}

} // namespace gicp_localizer

#endif // GICP_LOCALIZER_POINT_BUDGET_HPP
