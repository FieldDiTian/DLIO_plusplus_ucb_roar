#include "gicp_interface/detail/localizer_utils.hpp"

#include <algorithm>
#include <cmath>

void gicp_localizer::GicpLocalizer::publishGicpOdom(
    const nav_msgs::msg::Odometry &odom_msg) {
  auto output = odom_msg;

  // The observer supplies a velocity estimate but does not maintain a
  // statistically calibrated twist covariance. Follow race_common's existing
  // convention for unmeasured components instead of leaving the diagonal at
  // zero, which downstream filters can misread as perfect certainty.
  constexpr double kUnestimatedVariance = 1e6;
  auto &twist_covariance = output.twist.covariance;
  twist_covariance.fill(0.0);
  for (const size_t diagonal : {size_t{0}, size_t{7}, size_t{14},
                                size_t{21}, size_t{28}, size_t{35}}) {
    twist_covariance[diagonal] = kUnestimatedVariance;
  }

  this->localized_odom_pub->publish(output);
}

void gicp_localizer::GicpLocalizer::publishNavSatFix(
    const nav_msgs::msg::Odometry &odom_msg) {
  sensor_msgs::msg::NavSatFix fix;
  fix.header = odom_msg.header;
  fix.header.frame_id = this->base_frame;
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix.latitude = std::numeric_limits<double>::quiet_NaN();
  fix.longitude = std::numeric_limits<double>::quiet_NaN();
  fix.altitude = std::numeric_limits<double>::quiet_NaN();
  fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  if (this->navsat_origin_valid_) {
    const auto &p = odom_msg.pose.pose.position;
    if (navsatfix::enuToWgs84(p.x, p.y, p.z, this->navsat_origin_,
                              fix.latitude, fix.longitude, fix.altitude)) {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      fix.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
      // NavSatFix covariance is expressed in the local ENU tangent frame.
      fix.position_covariance[0] = odom_msg.pose.covariance[0];
      fix.position_covariance[4] = odom_msg.pose.covariance[7];
      fix.position_covariance[8] = odom_msg.pose.covariance[14];
    }
  } else {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "NavSatFix product is active but no ENU datum is configured; publishing "
        "STATUS_NO_FIX. Set localization/expected_enu_origin to "
        "lat_deg,lon_deg,alt_m.");
  }
  this->navsatfix_pub->publish(fix);
}

void gicp_localizer::GicpLocalizer::publishPose() {

  std::lock_guard<std::mutex> lock(this->pose_mutex);

  // [P2 FIX 2026-07-10e] Publish with the timestamp the pose is actually
  // VALID at. current_pose is a median-point-time quantity on accept/reject
  // frames and a scan-header-time quantity after a GT snap —
  // base_pose_stamp_ tracks exactly that (median time on accept/reject,
  // scan_stamp on snap). Stamping everything with the raw header time skewed
  // TF/path/pose consumers by ~half a sweep (~50 ms: 1.5 deg at 30 deg/s,
  // 3 m at 60 m/s for anyone interpolating against another sensor).
  // [P3 FIX 2026-07-14] Snapshot base_pose_stamp_ under seed_mtx_ (its owner
  // lock): the accept/reject/snap paths write it under seed_mtx_ on the scan
  // thread while this publisher runs on another thread holding only pose_mutex.
  // Lock order pose -> seed matches every other site.
  double base_pose_stamp_snapshot;
  Eigen::Vector3f accepted_measurement_position;
  Eigen::Quaternionf accepted_measurement_orientation;
  Eigen::Vector3f scan_velocity_world;
  {
    std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
    base_pose_stamp_snapshot = this->base_pose_stamp_;
    accepted_measurement_position = this->basePose.p;
    accepted_measurement_orientation = this->basePose.q.normalized();
    scan_velocity_world = this->prev_vel;
  }
  // Every accepted GICP is a valid measurement and must produce one product
  // frame. Start with that exact scan-time result. A trustworthy bounded IMU
  // bridge may advance it; an unavailable/rejected bridge never suppresses or
  // disguises the already-computed localization solution.
  double output_stamp_s = base_pose_stamp_snapshot;
  Eigen::Vector3f position = accepted_measurement_position;
  Eigen::Quaternionf orientation = accepted_measurement_orientation;
  Eigen::Vector3f output_velocity_body =
      accepted_measurement_orientation.conjugate() * scan_velocity_world;
  Eigen::Vector3f output_angular_velocity_body = Eigen::Vector3f::Zero();
  Eigen::Vector3f measurement_position = accepted_measurement_position;
  Eigen::Quaternionf measurement_orientation =
      accepted_measurement_orientation;
  double compensation_duration_s = 0.0;
  bool stationary_output = false;
  bool compensated_output = false;
  uint64_t output_observer_epoch = 0;
  std::string compensation_status = "raw_measurement";
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    const auto &snapshot = this->latency_compensated_output_;
    output_observer_epoch = snapshot.observer_epoch;
    if (!snapshot.status.empty()) compensation_status = snapshot.status;
    double delay_s = 0.0;
    const bool same_measurement =
        snapshot.valid && base_pose_stamp_snapshot > 0.0 &&
        std::abs(snapshot.measurement_stamp - base_pose_stamp_snapshot) < 1e-4;
    if (same_measurement && observerDelayWithinBound(
                                snapshot.measurement_stamp,
                                snapshot.output_stamp,
                                this->output_max_imu_delay_compensation_s_,
                                &delay_s) &&
        snapshot.measurement_p.allFinite() &&
        snapshot.measurement_q.coeffs().allFinite() &&
        snapshot.p.allFinite() && snapshot.q.coeffs().allFinite() &&
        snapshot.v_lin_body.allFinite() && snapshot.v_ang_body.allFinite()) {
      measurement_position = snapshot.measurement_p;
      measurement_orientation = snapshot.measurement_q.normalized();
      position = snapshot.p;
      orientation = snapshot.q.normalized();
      output_velocity_body = snapshot.v_lin_body;
      output_angular_velocity_body = snapshot.v_ang_body;
      output_stamp_s = snapshot.output_stamp;
      compensation_duration_s = delay_s;
      stationary_output = snapshot.stationary;
      compensated_output = true;
      output_observer_epoch = snapshot.observer_epoch;
      compensation_status = snapshot.status;
      RCLCPP_DEBUG(this->get_logger(),
                   "Accepted GICP output advanced %.1fms to latest IMU stamp",
                   1000.0 * delay_s);
    }
  }

  const bool have_raw_measurement =
      base_pose_stamp_snapshot > 0.0 &&
      accepted_measurement_position.allFinite() &&
      accepted_measurement_orientation.coeffs().allFinite();
  if (!have_raw_measurement) {
    RCLCPP_ERROR(this->get_logger(),
                 "Accepted GICP measurement is non-finite or unstamped; "
                 "cannot publish product output");
    return;
  }
  if (!compensated_output) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Accepted GICP latency compensation rejected (%s); publishing the "
        "legal raw measurement at scan time instead",
        compensation_status.c_str());
  }

  const rclcpp::Time pub_stamp(
      static_cast<int64_t>(output_stamp_s * 1e9),
      this->scan_stamp.get_clock_type());

  // Build the common product pose. It is carried by Odometry; the equivalent
  // PoseStamped remains debug-only below in the accepted-pair record.
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = pub_stamp;
  pose_msg.header.frame_id = this->map_frame;
  pose_msg.pose.position.x = position.x();
  pose_msg.pose.position.y = position.y();
  pose_msg.pose.position.z = position.z();
  pose_msg.pose.orientation.w = orientation.w();
  pose_msg.pose.orientation.x = orientation.x();
  pose_msg.pose.orientation.y = orientation.y();
  pose_msg.pose.orientation.z = orientation.z();

  // Publish exactly the same pose contract as Odometry.
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = pose_msg.header;
  odom_msg.child_frame_id = this->base_frame;
  odom_msg.pose.pose = pose_msg.pose;

  // REP-105: Odometry twist is expressed in child_frame_id (base_frame).
  odom_msg.twist.twist.linear.x = output_velocity_body.x();
  odom_msg.twist.twist.linear.y = output_velocity_body.y();
  odom_msg.twist.twist.linear.z = output_velocity_body.z();
  odom_msg.twist.twist.angular.x = output_angular_velocity_body.x();
  odom_msg.twist.twist.angular.y = output_angular_velocity_body.y();
  odom_msg.twist.twist.angular.z = output_angular_velocity_body.z();

  constexpr double kBaseSigmaXY = 0.05;
  constexpr double kBaseSigmaZ = 0.10;
  constexpr double kBaseSigmaRot = 0.01;
  double sigma_xy = kBaseSigmaXY;
  double sigma_z = kBaseSigmaZ;
  double sigma_rot = kBaseSigmaRot;
  if (this->last_gicp_valid_ &&
      std::isfinite(this->last_accepted_fitness_score_) &&
      this->last_accepted_fitness_score_ >= 0.0) {
    const double fitness_sigma = std::sqrt(this->last_accepted_fitness_score_);
    sigma_xy = std::max(kBaseSigmaXY, fitness_sigma);
    sigma_z = std::max(kBaseSigmaZ, 2.0 * fitness_sigma);
    sigma_rot = std::max(kBaseSigmaRot, 0.1 * fitness_sigma);
  }
  if (this->consecutive_failures_ > 0) {
    const double elapsed_dr =
        (this->last_accepted_scan_stamp_ > 0.0)
            ? std::max(0.0,
                       pub_stamp.seconds() - this->last_accepted_scan_stamp_)
            : 0.0;
    const double drift =
        this->dr_cov_time_rate_ * elapsed_dr +
        this->dr_cov_dist_frac_ *
            static_cast<double>(scan_velocity_world.norm()) * elapsed_dr;
    sigma_xy += drift;
    sigma_z += 2.0 * drift;
    sigma_rot += 0.05 * drift;
  }
  auto &pose_covariance = odom_msg.pose.covariance;
  pose_covariance.fill(0.0);
  pose_covariance[0] = sigma_xy * sigma_xy;
  pose_covariance[7] = sigma_xy * sigma_xy;
  pose_covariance[14] = sigma_z * sigma_z;
  pose_covariance[21] = sigma_rot * sigma_rot;
  pose_covariance[28] = sigma_rot * sigma_rot;
  pose_covariance[35] = sigma_rot * sigma_rot;
  this->publishGicpOdom(odom_msg);
  this->publishNavSatFix(odom_msg);
}
