#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <vector>

namespace gicp_localizer::detail {

struct ImuSample {
  double stamp_s{0.0};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  // Optional absolute attitude carried by an IMU/INS message. A plain raw IMU
  // leaves these empty and the estimator falls back to gyro propagation.
  std::optional<Eigen::Quaterniond> orientation_world_from_body;
  std::optional<Eigen::Vector3d> orientation_variance_rad2;
};

struct InertialState {
  double stamp_s{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  std::uint64_t epoch{0};
};

class ImuTrajectory {
public:
  // The first sample must have the same timestamp as the anchor state. Samples
  // must be finite and strictly increasing after that. Midpoint integration is
  // used over every interval.
  static ImuTrajectory
  integrate(const InertialState &anchor, const std::vector<ImuSample> &samples,
            const Eigen::Vector3d &gravity_world = Eigen::Vector3d(0.0, 0.0,
                                                                   -9.80665));

  bool empty() const noexcept;
  double start_time_s() const;
  double end_time_s() const;
  std::uint64_t epoch() const;

  InertialState state_at(double stamp_s) const;

  // The point must already be expressed in the common vehicle/CG frame at its
  // acquisition time. The result is expressed in the vehicle/CG frame at the
  // requested reference time.
  Eigen::Vector3d deskew_point(const Eigen::Vector3d &point_at_acquisition,
                               double acquisition_time_s,
                               double reference_time_s) const;

  const std::vector<InertialState> &states() const noexcept;

private:
  std::vector<InertialState> states_;
};

} // namespace gicp_localizer::detail
