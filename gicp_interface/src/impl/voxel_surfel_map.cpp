#include "gicp_interface/detail/voxel_surfel_map.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace gicp_localizer::detail {
namespace {

struct Accumulator {
  std::size_t count{0};
  Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d centered_sum{Eigen::Matrix3d::Zero()};

  void add(const Eigen::Vector3d &point) {
    ++count;
    const Eigen::Vector3d delta = point - mean;
    mean += delta / static_cast<double>(count);
    centered_sum += delta * (point - mean).transpose();
  }
};

VoxelKey voxel_key(const Eigen::Vector3d &point, double voxel_size_m) {
  return VoxelKey{
      static_cast<std::int64_t>(std::floor(point.x() / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(point.y() / voxel_size_m)),
      static_cast<std::int64_t>(std::floor(point.z() / voxel_size_m))};
}

std::size_t mix_hash(std::size_t seed, std::uint64_t value) {
  value ^= value >> 30U;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27U;
  value *= 0x94d049bb133111ebULL;
  value ^= value >> 31U;
  return seed ^ (static_cast<std::size_t>(value) + 0x9e3779b97f4a7c15ULL +
                 (seed << 6U) + (seed >> 2U));
}

} // namespace

bool VoxelKey::operator==(const VoxelKey &other) const noexcept {
  return x == other.x && y == other.y && z == other.z;
}

std::size_t VoxelKeyHash::operator()(const VoxelKey &key) const noexcept {
  std::size_t seed = 0;
  seed = mix_hash(seed, static_cast<std::uint64_t>(key.x));
  seed = mix_hash(seed, static_cast<std::uint64_t>(key.y));
  return mix_hash(seed, static_cast<std::uint64_t>(key.z));
}

VoxelSurfelMap VoxelSurfelMap::build(const std::vector<Eigen::Vector3d> &points,
                                     const SurfelMapConfig &config) {
  if (!std::isfinite(config.voxel_size_m) || config.voxel_size_m <= 0.0 ||
      config.minimum_points < 3 ||
      !std::isfinite(config.maximum_normalized_smallest_eigenvalue) ||
      config.maximum_normalized_smallest_eigenvalue < 0.0 ||
      config.maximum_normalized_smallest_eigenvalue >= 1.0 ||
      !std::isfinite(config.minimum_middle_to_largest_eigenvalue) ||
      config.minimum_middle_to_largest_eigenvalue < 0.0 ||
      config.minimum_middle_to_largest_eigenvalue > 1.0 ||
      !std::isfinite(config.minimum_largest_eigenvalue) ||
      config.minimum_largest_eigenvalue < 0.0) {
    throw std::invalid_argument("invalid voxel-surfel map configuration");
  }

  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> accumulators;
  accumulators.reserve(points.size() /
                       std::max<std::size_t>(config.minimum_points, 1));
  for (const Eigen::Vector3d &point : points) {
    if (point.allFinite()) {
      accumulators[voxel_key(point, config.voxel_size_m)].add(point);
    }
  }

  VoxelSurfelMap map;
  map.config_ = config;
  map.surfels_.reserve(accumulators.size());
  for (const auto &entry : accumulators) {
    const Accumulator &accumulator = entry.second;
    if (accumulator.count < config.minimum_points) {
      continue;
    }
    const Eigen::Matrix3d covariance =
        accumulator.centered_sum / static_cast<double>(accumulator.count - 1);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
      continue;
    }
    const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    const double eigenvalue_sum = eigenvalues.sum();
    if (eigenvalue_sum <= 0.0 ||
        eigenvalues.z() < config.minimum_largest_eigenvalue ||
        eigenvalues.x() / eigenvalue_sum >
            config.maximum_normalized_smallest_eigenvalue ||
        eigenvalues.y() / eigenvalues.z() <
            config.minimum_middle_to_largest_eigenvalue) {
      continue;
    }

    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    if (normal.z() < 0.0) {
      normal = -normal;
    }
    map.surfels_.emplace(entry.first,
                         Surfel{accumulator.mean, normal, covariance,
                                eigenvalues, accumulator.count});
  }
  return map;
}

VoxelSurfelMap VoxelSurfelMap::from_precomputed(
    const SurfelMapConfig &config,
    std::unordered_map<VoxelKey, Surfel, VoxelKeyHash> surfels) {
  if (!std::isfinite(config.voxel_size_m) || config.voxel_size_m <= 0.0 ||
      surfels.empty()) {
    throw std::invalid_argument(
        "precomputed surfel map must be non-empty with valid voxel size");
  }
  for (const auto &entry : surfels) {
    const Surfel &surfel = entry.second;
    if (!surfel.mean.allFinite() || !surfel.normal.allFinite() ||
        std::abs(surfel.normal.norm() - 1.0) > 1e-6 ||
        !surfel.eigenvalues.allFinite() || surfel.point_count < 3) {
      throw std::invalid_argument("precomputed surfel is invalid");
    }
  }
  VoxelSurfelMap map;
  map.config_ = config;
  map.surfels_ = std::move(surfels);
  return map;
}

std::optional<SurfelMatch>
VoxelSurfelMap::nearest_surfel(const Eigen::Vector3d &point_world,
                               double maximum_plane_distance_m,
                               double maximum_centroid_distance_m) const {
  if (!point_world.allFinite() || !std::isfinite(maximum_plane_distance_m) ||
      !std::isfinite(maximum_centroid_distance_m) ||
      maximum_plane_distance_m <= 0.0 || maximum_centroid_distance_m <= 0.0) {
    throw std::invalid_argument("invalid surfel query");
  }

  const VoxelKey center = voxel_key(point_world, config_.voxel_size_m);
  const int radius =
      std::max(1, static_cast<int>(std::ceil(maximum_centroid_distance_m /
                                             config_.voxel_size_m)));
  std::optional<SurfelMatch> best;
  double best_score = std::numeric_limits<double>::infinity();
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        const auto found = surfels_.find(
            VoxelKey{center.x + dx, center.y + dy, center.z + dz});
        if (found == surfels_.end()) {
          continue;
        }
        const Surfel &surfel = found->second;
        const Eigen::Vector3d delta = point_world - surfel.mean;
        const double centroid_distance = delta.norm();
        const double plane_distance = surfel.normal.dot(delta);
        if (centroid_distance > maximum_centroid_distance_m ||
            std::abs(plane_distance) > maximum_plane_distance_m) {
          continue;
        }
        const double score =
            std::abs(plane_distance) + 0.05 * centroid_distance;
        if (score < best_score) {
          best_score = score;
          best = SurfelMatch{&surfel, plane_distance, centroid_distance};
        }
      }
    }
  }
  return best;
}

std::size_t VoxelSurfelMap::size() const noexcept { return surfels_.size(); }

double VoxelSurfelMap::voxel_size_m() const noexcept {
  return config_.voxel_size_m;
}

const SurfelMapConfig &VoxelSurfelMap::config() const noexcept {
  return config_;
}

const std::unordered_map<VoxelKey, Surfel, VoxelKeyHash> &
VoxelSurfelMap::surfels() const noexcept {
  return surfels_;
}

} // namespace gicp_localizer::detail
