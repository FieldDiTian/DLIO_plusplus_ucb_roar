#pragma once

#include "gicp_interface/detail/delayed_ieskf.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <memory>

namespace gicp_localizer {

// Backend-neutral motion state used around small_gicp.  The scan matcher still
// owns correspondence search and candidate validation; this class owns the
// timestamped inertial state and applies an accepted map pose at the scan time
// before replaying newer IMU/wheel observations.
struct DelayedGicpFusionConfig {
  double history_duration_s{1.0};
  double pose_xy_sigma_m{0.10};
  double pose_yaw_sigma_rad{0.5 * 3.14159265358979323846 / 180.0};
  double minimum_information_eigenvalue{1e-6};
  gicp_localizer::detail::ImuNoise imu_noise{};
};

struct DelayedGicpFusionResult {
  gicp_localizer::detail::FilterState measurement_time_state;
  gicp_localizer::detail::FilterState latest_state;
  Eigen::Vector3d planar_innovation{Eigen::Vector3d::Zero()};
  std::size_t replayed_imu_samples{0};
  double delay_s{0.0};
};

// Re-express a world-frame velocity after an absolute attitude re-anchor while
// preserving the physically observed body-frame velocity components.  This is
// required when a recovery seed is brought from scan time to the latest IMU
// attitude in a fast corner.
Eigen::Vector3d reexpressVelocityPreservingBodyComponents(
    const Eigen::Quaterniond &previous_world_from_body,
    const Eigen::Quaterniond &updated_world_from_body,
    const Eigen::Vector3d &previous_velocity_world);

class DelayedGicpFusion {
public:
  explicit DelayedGicpFusion(
      const DelayedGicpFusionConfig &config = DelayedGicpFusionConfig{});

  void reset(const gicp_localizer::detail::FilterState &state,
             const gicp_localizer::detail::ImuSample &sample);
  bool initialized() const noexcept;

  void push_imu(const gicp_localizer::detail::ImuSample &sample);
  gicp_localizer::detail::PlanarVelocityUpdateResult
  apply_body_planar_velocity(const Eigen::Vector2d &velocity_body_mps,
                             const Eigen::Vector2d &variance_mps2);

  DelayedGicpFusionResult
  apply_planar_pose(double measurement_time_s,
                    const Eigen::Isometry3d &world_from_body_measurement);

  const gicp_localizer::detail::FilterState &state() const;
  gicp_localizer::detail::FilterState state_at(double stamp_s) const;
  double newest_time_s() const;
  double oldest_time_s() const;

private:
  DelayedGicpFusionConfig config_;
  std::unique_ptr<gicp_localizer::detail::DelayedIeskf> filter_;
};

} // namespace gicp_localizer
