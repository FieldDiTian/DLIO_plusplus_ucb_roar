#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace gicp_localizer::detail {

struct VoxelKey {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelKey &other) const noexcept;
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const noexcept;
};

struct Surfel {
  Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d eigenvalues{Eigen::Vector3d::Zero()};
  std::size_t point_count{0};
};

struct SurfelMapConfig {
  double voxel_size_m{0.5};
  std::size_t minimum_points{6};
  double maximum_normalized_smallest_eigenvalue{0.12};
  double minimum_middle_to_largest_eigenvalue{0.02};
  double minimum_largest_eigenvalue{1e-4};
};

struct SurfelMatch {
  const Surfel *surfel{nullptr};
  double signed_plane_distance_m{0.0};
  double centroid_distance_m{0.0};
};

class VoxelSurfelMap {
public:
  static VoxelSurfelMap
  build(const std::vector<Eigen::Vector3d> &points,
        const SurfelMapConfig &config = SurfelMapConfig{});

  static VoxelSurfelMap
  from_precomputed(const SurfelMapConfig &config,
                   std::unordered_map<VoxelKey, Surfel, VoxelKeyHash> surfels);

  std::optional<SurfelMatch>
  nearest_surfel(const Eigen::Vector3d &point_world,
                 double maximum_plane_distance_m,
                 double maximum_centroid_distance_m) const;

  std::size_t size() const noexcept;
  double voxel_size_m() const noexcept;
  const SurfelMapConfig &config() const noexcept;
  const std::unordered_map<VoxelKey, Surfel, VoxelKeyHash> &
  surfels() const noexcept;

private:
  SurfelMapConfig config_;
  std::unordered_map<VoxelKey, Surfel, VoxelKeyHash> surfels_;
};

} // namespace gicp_localizer::detail
