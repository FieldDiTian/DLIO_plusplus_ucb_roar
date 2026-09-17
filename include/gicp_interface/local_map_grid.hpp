#ifndef GICP_LOCALIZER__LOCAL_MAP_GRID_HPP_
#define GICP_LOCALIZER__LOCAL_MAP_GRID_HPP_

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace gicp_localizer {

// Immutable XY spatial index for extracting circular local-map targets.  The
// grid stores point indices rather than duplicate points, allowing the target
// builder to reuse covariances precomputed on the complete map.
template <typename PointT>
class LocalMapGrid {
 public:
  explicit LocalMapGrid(double cell_size_m = 50.0)
      : cell_size_m_(cell_size_m) {
    if (!std::isfinite(cell_size_m_) || cell_size_m_ <= 0.0) {
      throw std::invalid_argument("local-map grid cell size must be positive");
    }
  }

  template <typename CloudT>
  void build(const CloudT& cloud) {
    cells_.clear();
    indexed_points_ = 0;
    for (size_t index = 0; index < cloud.size(); ++index) {
      const auto& point = cloud.points[index];
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        continue;
      }
      cells_[key(cell(point.x), cell(point.y))].push_back(
          static_cast<uint32_t>(index));
      ++indexed_points_;
    }
  }

  template <typename CloudT>
  std::vector<size_t> queryCircle(const CloudT& cloud, double center_x,
                                  double center_y, double radius_m) const {
    std::vector<size_t> indices;
    if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
        !std::isfinite(radius_m) || radius_m <= 0.0) {
      return indices;
    }

    const int32_t min_x = cell(center_x - radius_m);
    const int32_t max_x = cell(center_x + radius_m);
    const int32_t min_y = cell(center_y - radius_m);
    const int32_t max_y = cell(center_y + radius_m);
    const double radius_sq = radius_m * radius_m;
    for (int32_t x = min_x; x <= max_x; ++x) {
      for (int32_t y = min_y; y <= max_y; ++y) {
        const auto it = cells_.find(key(x, y));
        if (it == cells_.end()) {
          continue;
        }
        for (const uint32_t index : it->second) {
          if (index >= cloud.size()) {
            continue;
          }
          const auto& point = cloud.points[index];
          const double dx = static_cast<double>(point.x) - center_x;
          const double dy = static_cast<double>(point.y) - center_y;
          if (dx * dx + dy * dy <= radius_sq) {
            indices.push_back(static_cast<size_t>(index));
          }
        }
      }
    }
    return indices;
  }

  size_t indexedPointCount() const { return indexed_points_; }
  size_t cellCount() const { return cells_.size(); }
  double cellSize() const { return cell_size_m_; }

 private:
  int32_t cell(double coordinate) const {
    return static_cast<int32_t>(std::floor(coordinate / cell_size_m_));
  }

  static int64_t key(int32_t x, int32_t y) {
    const uint64_t ux = static_cast<uint32_t>(x);
    const uint64_t uy = static_cast<uint32_t>(y);
    return static_cast<int64_t>((ux << 32U) | uy);
  }

  double cell_size_m_;
  size_t indexed_points_ = 0;
  std::unordered_map<int64_t, std::vector<uint32_t>> cells_;
};

}  // namespace gicp_localizer

#endif  // GICP_LOCALIZER__LOCAL_MAP_GRID_HPP_
