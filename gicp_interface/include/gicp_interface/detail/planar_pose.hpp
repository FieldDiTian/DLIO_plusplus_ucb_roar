#pragma once

#include <Eigen/Geometry>

namespace gicp_localizer::detail {

struct PlanarPose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

double normalize_yaw(double yaw);

PlanarPose extract_planar_pose(const Eigen::Isometry3d &pose);

// Replaces world-frame x, y, and yaw while preserving propagated z and the
// roll/pitch component needed for LiDAR deskew.
Eigen::Isometry3d inject_planar_pose(const Eigen::Isometry3d &propagated_pose,
                                     const PlanarPose &map_measurement);

} // namespace gicp_localizer::detail
