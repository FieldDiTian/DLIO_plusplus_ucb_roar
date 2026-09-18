#include "gicp_interface/detail/planar_pose.hpp"

#include <cmath>
#include <stdexcept>

namespace gicp_localizer::detail {
namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

Eigen::Matrix3d yaw_rotation(double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

void require_finite(const PlanarPose &pose) {
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
      !std::isfinite(pose.yaw)) {
    throw std::invalid_argument("planar pose must be finite");
  }
}

} // namespace

double normalize_yaw(double yaw) {
  if (!std::isfinite(yaw)) {
    throw std::invalid_argument("yaw must be finite");
  }
  double wrapped = std::fmod(yaw + 3.14159265358979323846, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - 3.14159265358979323846;
}

PlanarPose extract_planar_pose(const Eigen::Isometry3d &pose) {
  if (!pose.matrix().allFinite()) {
    throw std::invalid_argument("pose must be finite");
  }

  const Eigen::Matrix3d &rotation = pose.linear();
  return PlanarPose{pose.translation().x(), pose.translation().y(),
                    std::atan2(rotation(1, 0), rotation(0, 0))};
}

Eigen::Isometry3d inject_planar_pose(const Eigen::Isometry3d &propagated_pose,
                                     const PlanarPose &map_measurement) {
  if (!propagated_pose.matrix().allFinite()) {
    throw std::invalid_argument("propagated pose must be finite");
  }
  require_finite(map_measurement);

  const double propagated_yaw = extract_planar_pose(propagated_pose).yaw;
  const Eigen::Matrix3d roll_pitch_component =
      yaw_rotation(-propagated_yaw) * propagated_pose.linear();

  Eigen::Isometry3d corrected = propagated_pose;
  corrected.translation().x() = map_measurement.x;
  corrected.translation().y() = map_measurement.y;
  corrected.linear() =
      yaw_rotation(normalize_yaw(map_measurement.yaw)) * roll_pitch_component;
  return corrected;
}

} // namespace gicp_localizer::detail
