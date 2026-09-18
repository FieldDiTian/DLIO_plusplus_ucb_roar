#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <small_gicp/points/traits.hpp>
#include <small_gicp/util/fast_floor.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace gicp_localizer {

// Deterministic counterpart to small_gicp::voxelgrid_sampling_tbb().  The
// upstream parallel implementation reduces independent fixed-size blocks.  A
// voxel that straddles a block boundary is therefore emitted more than once,
// and atomic output allocation makes the resulting point order run-dependent.
//
// This implementation keeps the expensive coordinate generation, sorting and
// centroid calculation parallel, but gives every input point a unique total
// order (voxel key, original index).  A short serial pass identifies exact
// voxel boundaries, after which each voxel is reduced independently into its
// deterministic key-order output slot.
template <typename InputPointCloud, typename OutputPointCloud = InputPointCloud>
std::shared_ptr<OutputPointCloud> deterministicVoxelgridSampling(
    const InputPointCloud& points, double leaf_size) {
  if (!std::isfinite(leaf_size) || leaf_size <= 0.0) {
    throw std::invalid_argument("voxel leaf size must be finite and positive");
  }

  const size_t input_size = small_gicp::traits::size(points);
  if (input_size == 0) {
    return std::make_shared<OutputPointCloud>();
  }

  const double inv_leaf_size = 1.0 / leaf_size;
  constexpr std::uint64_t kInvalidCoord =
      std::numeric_limits<std::uint64_t>::max();
  constexpr int kCoordBitSize = 21;
  constexpr int kCoordOffset = 1 << (kCoordBitSize - 1);
  constexpr std::uint64_t kCoordBitMask = (std::uint64_t{1} << 21) - 1;

  using KeyedIndex = std::pair<std::uint64_t, size_t>;
  std::vector<KeyedIndex> keyed_points(input_size);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, input_size, 4096),
      [&](const tbb::blocked_range<size_t>& range) {
        for (size_t i = range.begin(); i != range.end(); ++i) {
          const Eigen::Vector4d point = small_gicp::traits::point(points, i);
          if (!point.head<3>().allFinite()) {
            keyed_points[i] = {kInvalidCoord, i};
            continue;
          }

          const Eigen::Array4i coord =
              small_gicp::fast_floor(point * inv_leaf_size) + kCoordOffset;
          if ((coord < 0).any() ||
              (coord > static_cast<int>(kCoordBitMask)).any()) {
            keyed_points[i] = {kInvalidCoord, i};
            continue;
          }

          const std::uint64_t key =
              (static_cast<std::uint64_t>(coord[0]) << (kCoordBitSize * 0)) |
              (static_cast<std::uint64_t>(coord[1]) << (kCoordBitSize * 1)) |
              (static_cast<std::uint64_t>(coord[2]) << (kCoordBitSize * 2));
          keyed_points[i] = {key, i};
        }
      });

  // Pair ordering compares key first and original index second.  Since the
  // index is unique, this is a total order even if parallel_sort is unstable.
  tbb::parallel_sort(keyed_points.begin(), keyed_points.end());

  size_t valid_size = 0;
  while (valid_size < keyed_points.size() &&
         keyed_points[valid_size].first != kInvalidCoord) {
    ++valid_size;
  }

  auto downsampled = std::make_shared<OutputPointCloud>();
  if (valid_size == 0) {
    return downsampled;
  }

  std::vector<size_t> voxel_starts;
  voxel_starts.reserve(valid_size);
  voxel_starts.push_back(0);
  for (size_t i = 1; i < valid_size; ++i) {
    if (keyed_points[i - 1].first != keyed_points[i].first) {
      voxel_starts.push_back(i);
    }
  }
  voxel_starts.push_back(valid_size);  // sentinel

  const size_t output_size = voxel_starts.size() - 1;
  small_gicp::traits::resize(*downsampled, output_size);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, output_size, 256),
      [&](const tbb::blocked_range<size_t>& range) {
        for (size_t voxel = range.begin(); voxel != range.end(); ++voxel) {
          Eigen::Vector4d sum = Eigen::Vector4d::Zero();
          for (size_t i = voxel_starts[voxel];
               i < voxel_starts[voxel + 1]; ++i) {
            sum += small_gicp::traits::point(points, keyed_points[i].second);
          }
          small_gicp::traits::set_point(
              *downsampled, voxel, sum / sum.w());
        }
      });

  return downsampled;
}

}  // namespace gicp_localizer
