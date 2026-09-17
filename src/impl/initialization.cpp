#include "gicp_interface/detail/localizer_utils.hpp"

void gicp_localizer::GicpLocalizer::applyInitialPoseFromParams() {

  if (!this->use_param_initial_pose_) {
    return;
  }

  Eigen::Vector3f position(static_cast<float>(this->initial_pose_x_),
                           static_cast<float>(this->initial_pose_y_),
                           static_cast<float>(this->initial_pose_z_));

  Eigen::AngleAxisf roll_angle(static_cast<float>(this->initial_pose_roll_),
                               Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(static_cast<float>(this->initial_pose_pitch_),
                                Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(static_cast<float>(this->initial_pose_yaw_),
                              Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf orientation = yaw_angle * pitch_angle * roll_angle;
  orientation.normalize();

  // Pose values may be supplied in the LiDAR frame (matching GLIM's
  // traj_lidar.txt), but internally this node tracks the base_link pose in
  // world. When frame=="lidar", convert T_world_lidar -> T_world_base by
  // post-multiplying inv(baselink2lidar_T). This requires the URDF TF from
  // robot_state_publisher; defer until the first pointcloud if not yet cached.
  if (this->initial_pose_frame_ == "lidar") {
    if (!this->extrinsics_cached_) {
      this->pending_initial_pose_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Initial pose in lidar frame; deferring apply until "
                  "baselink->lidar TF is cached");
      return;
    }
    const Eigen::Matrix3f &R_bl = this->extrinsics.baselink2lidar.R;
    const Eigen::Vector3f &t_bl = this->extrinsics.baselink2lidar.t;
    // T_world_base.t = R_world_lidar * (-R_base_lidar^T * t_base_lidar) +
    // t_world_lidar With lidar == luminar_front (no rotation in URDF) this
    // simplifies to t_world_base = t_world_lidar - R_world_lidar * R_base_lidar
    // * (R_base_lidar^T * t_base_lidar). We compute the general form:
    // T_world_base = T_world_lidar * inv(T_base_lidar).
    Eigen::Matrix3f R_lb = R_bl.transpose();
    Eigen::Vector3f t_lb = -R_lb * t_bl;
    Eigen::Matrix3f R_world_lidar = orientation.toRotationMatrix();
    Eigen::Vector3f position_base = R_world_lidar * t_lb + position;
    Eigen::Quaternionf orientation_base(R_world_lidar * R_lb);
    orientation_base.normalize();
    RCLCPP_INFO(this->get_logger(),
                "Converted initial lidar pose -> base_link: [%.3f, %.3f, %.3f] "
                "(lidar) -> [%.3f, %.3f, %.3f] (base)",
                position.x(), position.y(), position.z(), position_base.x(),
                position_base.y(), position_base.z());
    position = position_base;
    orientation = orientation_base;
  }
  this->pending_initial_pose_ = false;

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = position;
    // [REVIEW FIX 2026-07-08 P3] initialized is published LAST (below), after
    // basePose/observer state are written — a MultiThreadedExecutor scan
    // callback could otherwise pass the atomic initialized check and start
    // deskew from a stale/default basePose.
  }

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = position;
    this->state.q = orientation;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.v.lin.b = Eigen::Vector3f::Zero();
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    // [REVIEW FIX 2026-07-08 P3] Preserve calibrated IMU biases: reinit moves
    // the POSE estimate, it does not invalidate the sensor calibration.
    // Zeroing here while imu_calibrated_ stayed true meant callbackImu kept
    // subtracting a zero bias — calibration silently stopped being used.
    if (!this->imu_calibrated_.load()) {
      this->state.b.accel = Eigen::Vector3f::Zero();
      this->state.b.gyro = Eigen::Vector3f::Zero();
    }
    this->geo.prev_p = position;
    this->geo.prev_q = orientation;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    ++this->observer_epoch_;
    this->observer_pose_history_.clear();
    this->latency_compensated_output_.valid = false;
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  {
    std::lock_guard<std::mutex> seed_lock(
        this->seed_mtx_); // [P2 FIX 2026-07-09]
    this->basePose.p = position;
    this->basePose.q = orientation;
    this->base_pose_stamp_ = 0.0; // parameter pose has no timestamp
    this->prev_vel.setZero();
    this->T_prior_velocity_.setZero();
  }
  if (this->gicp_dof_mode_ == "planar" && this->track_elevation_enabled_) {
    this->track_z_hold_m_.store(static_cast<double>(position.z()),
                                std::memory_order_relaxed);
    this->track_z_hold_valid_.store(true, std::memory_order_release);
    this->track_output_z_m_.store(static_cast<double>(position.z()),
                                  std::memory_order_relaxed);
    this->track_output_z_valid_.store(true, std::memory_order_release);
  }
  this->initialized = true; // publish only after ALL pose state is consistent

  RCLCPP_INFO(this->get_logger(),
              "Initial pose loaded from parameters at [%.2f, %.2f, %.2f] m "
              "with RPY [%.2f, %.2f, %.2f] rad",
              this->initial_pose_x_, this->initial_pose_y_,
              this->initial_pose_z_, this->initial_pose_roll_,
              this->initial_pose_pitch_, this->initial_pose_yaw_);
}

void gicp_localizer::GicpLocalizer::resetEstimatorForEpochChangeLocked(
    const char *source, double regress_s) {
  RCLCPP_WARN(this->get_logger(),
              "EPOCH RESET (%s, rewind %.3f s): coordinated re-initialization "
              "of the estimator "
              "(buffers, timestamp seeds, calibration, observer). Output "
              "pauses until re-seeded on "
              "the new epoch.",
              source, regress_s);

  {
    // Canonical lock order: pose -> seed -> calib -> gt_odom -> geo -> imu.
    // A superset of every nested acquisition elsewhere, so no concurrent
    // callback can observe a partially-reset state and none can deadlock
    // against this.
    std::lock_guard<std::mutex> pose_lock(this->pose_mutex);
    std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
    std::lock_guard<std::mutex> calib_lock(this->calib_mtx_);

    {
      std::lock_guard<std::mutex> gt_lock(this->gt_odom_mtx_);
      this->gt_odom_buffer_.clear();
    }
    {
      std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
      this->imu_buffer.clear();
      // imu_meas is intentionally left as-is: it is overwritten by the next
      // buffered sample before any consumer reads it (propagateState is gated
      // off while initialized == false, set below).
    }

    // Timestamp seeds -> unknown (0), exactly as constructed. prev_scan_stamp=0
    // makes the next scan a clean "first scan" that re-anchors the scan chain
    // to the new epoch within a frame or two (no scan-queue surgery needed).
    this->prev_scan_stamp = 0.0;
    this->base_pose_stamp_ = 0.0;
    this->t_prior_stamp_ = 0.0;
    this->scan_stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    this->last_gicp_stamp_ =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    this->last_scan_time_span_s_ = -1.0;

    // Init / calibration state machine -> fresh (re-seed + re-calibrate on the
    // new epoch). Biases are re-estimated rather than carried across a physical
    // power-cycle.
    this->first_imu_received = false;
    this->first_imu_stamp_ = -1.0;
    this->imu_calibrated_ = false;
    this->init_phase_ = InitPhase::WAITING;
    this->imu_calib_start_stamp_ = -1.0;
    this->imu_calib_count_ = 0;
    this->imu_calib_gyro_sum_.setZero();
    this->imu_calib_accel_sum_.setZero();
    this->imu_calib_gyro_sq_sum_.setZero();
    this->rtk_calib_start_stamp_ = -1.0;
    this->rtk_calib_count_ = 0;
    this->rtk_gyro_bias_sum_.setZero();
    this->rtk_accel_bias_sum_.setZero();
    this->rtk_gyro_bias_sq_sum_.setZero();
    this->rtk_accel_bias_sq_sum_.setZero();
    this->has_prev_gt_for_accel_ = false;
    this->prev_gt_stamp_ = 0.0;
    this->prev_v_world_.setZero();
    this->has_latest_rtk_seed_ = false;

    // Accept / fitness tracking.
    this->last_gicp_valid_ = false;
    this->wheel_stationary_gate_active_ = false;
    this->accepted_gicp_count_.store(0, std::memory_order_relaxed);
    this->last_accepted_fitness_score_ = -1.0;
    this->last_accepted_scan_stamp_ = -1.0;
    this->consecutive_failures_ = 0;
    this->failure_streak_had_timeout_ = false;
    this->previous_failure_track_along_ = false;
    this->fitness_history_.clear();
    this->track_z_hold_valid_.store(false, std::memory_order_release);
    this->track_z_hold_m_.store(0.0, std::memory_order_relaxed);
    this->track_output_z_valid_.store(false, std::memory_order_release);
    this->track_output_z_m_.store(0.0, std::memory_order_relaxed);
    this->track_z_offset_valid_ = false;
    this->track_z_offset_m_ = 0.0;

    // Poses -> identity; the re-seed below (param pose / odom-init / first
    // scan) establishes the real new-epoch pose.
    this->current_pose = Eigen::Matrix4f::Identity();
    this->T_prior = Eigen::Matrix4f::Identity();
    this->last_gicp_pose_ = Eigen::Matrix4f::Identity();
    this->basePose.p = Eigen::Vector3f::Zero();
    this->basePose.q = Eigen::Quaternionf::Identity();
    this->prev_vel.setZero();
    this->T_prior_velocity_.setZero();

    {
      std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
      this->state.p.setZero();
      this->state.q = Eigen::Quaternionf::Identity();
      this->state.v.lin.w.setZero();
      this->state.v.lin.b.setZero();
      this->state.v.ang.w.setZero();
      this->state.v.ang.b.setZero();
      this->state.b.gyro.setZero();
      this->state.b.accel.setZero();
      this->geo.prev_p.setZero();
      this->geo.prev_q = Eigen::Quaternionf::Identity();
      this->geo.prev_vel.setZero();
      ++this->observer_epoch_;
      this->observer_pose_history_.clear();
      this->latency_compensated_output_.valid = false;
      this->geo.first_opt_done = false;
      this->geo.dp = 0.0;
      this->geo.dq_deg = 0.0;
      ++this->geo
            .update_seq; // discard any in-flight propagateState computation
    }

    // Re-seed sources: unseeded until the new epoch provides a seed.
    this->initialized = false;
    this->gt_odom_received_ = false;
    this->use_odom_init_applied_ = false;
  } // all estimator locks released here

  // [P2 FIX 2026-07-15] Discard the front synchronizer's old-epoch state too.
  // The estimator reset above does not touch primary_queue_ / aux buffers, so a
  // pre-reset front could otherwise be processed immediately after the reset
  // (as a fresh scan, re-seeding prev_scan_stamp to the OLD epoch) or merged
  // against stale old-epoch aux sweeps (a wrong-sweep concat). Clear both; the
  // discarded fronts are accounted via front_epoch_dropped_ so the conservation
  // invariant (checked in drainFrontSync) still holds. Lock order sync_mtx_ ->
  // aux.mtx matches the worker. Separate scope from the estimator locks above —
  // the two sets are independent, so keeping them unnested avoids entangling
  // the lock graph. The worker never holds sync_mtx_ while inside processScan
  // (it unlocks before calling it), so this cannot deadlock against the caller.
  if (this->sync_active_) {
    std::lock_guard<std::mutex> sync_lock(this->sync_mtx_);
    // Advance the admission generation before accepting any new front. A front
    // already popped by the worker retains the old generation and is rejected
    // at the scan-gate entry below instead of being run against reset state.
    this->sync_epoch_.fetch_add(1);
    this->registration_rate_limiter_.reset();
    while (!this->primary_queue_.empty()) {
      this->primary_queue_.pop_front();
      ++this->front_epoch_dropped_;
    }
    for (auto &auxp : this->aux_lidars_) {
      std::lock_guard<std::mutex> alk(auxp->mtx);
      auxp->buffer.clear();
    }
  }

  // Re-apply the epoch-agnostic param initial pose (if configured) so output
  // resumes immediately without waiting for GT / odom-init. Called OUTSIDE the
  // lock scope above because it takes pose_mutex/seed_mtx_ itself (self-guarded
  // by use_param_initial_pose_, so a no-op otherwise).
  this->applyInitialPoseFromParams();
}

void gicp_localizer::GicpLocalizer::applyInitialPose(
    const Eigen::Vector3f &p, const Eigen::Quaternionf &q_in,
    const rclcpp::Time &stamp, const std::string &source,
    const Eigen::Vector3f *v_world_lin) {

  Eigen::Quaternionf q = q_in;
  if (q.squaredNorm() < 1e-10f) {
    RCLCPP_WARN(this->get_logger(),
                "Received near-zero quaternion in initial pose, ignoring");
    return;
  }
  q.normalize();

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = p;
    // [REVIEW FIX 2026-07-08 P3] initialized is published LAST (below): with a
    // MultiThreadedExecutor, a scan callback could pass the atomic
    // initialized check here and start deskew from the stale/default basePose
    // that is only written further down.
    // [P2 FIX 2026-07-09] Overwrite scan_stamp only on the FIRST seed:
    // callbackPointCloud writes it on the scan thread without this mutex, so
    // a mid-run reinit retimed an in-flight scan's deskew/publish.
    if (stamp.nanoseconds() > 0 && !this->initialized.load()) {
      this->scan_stamp = stamp;
    }
  }

  const Eigen::Vector3f v0 =
      v_world_lin ? *v_world_lin : Eigen::Vector3f::Zero();

  // Reset geometric observer state to prevent drift from previous estimates
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = p;
    this->state.q = q;
    // [P3 FIX 2026-07-14] Seed linear velocity from the caller when provided
    // (GT odom-init carries a valid twist), else zero. Angular velocity is left
    // zero — the seed only knows a linear velocity.
    this->state.v.lin.w = v0;
    this->state.v.lin.b = q.conjugate() * v0;
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    // [REVIEW FIX 2026-07-08 P3] Same bias preservation as the param-pose
    // path — /initialpose (RViz) reinit must not discard RTK/stationary
    // calibration while imu_calibrated_ remains true.
    if (!this->imu_calibrated_.load()) {
      this->state.b.accel = Eigen::Vector3f::Zero();
      this->state.b.gyro = Eigen::Vector3f::Zero();
    }
    this->geo.prev_p = p;
    this->geo.prev_q = q;
    this->geo.prev_vel = v0;
    ++this->observer_epoch_;
    this->observer_pose_history_.clear();
    this->latency_compensated_output_.valid = false;
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  {
    // [P2 FIX 2026-07-09] Seed writes land atomically between scans.
    std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
    this->basePose.p = p;
    this->basePose.q = q;
    // [REVIEW FIX 2026-07-08] Valid at the provided stamp if there is one;
    // otherwise unknown (0) -> the INS prior falls back to prev_scan_stamp.
    this->base_pose_stamp_ = (stamp.nanoseconds() > 0) ? stamp.seconds() : 0.0;
    this->prev_vel = v0;
    this->T_prior_velocity_ = v0;
  }
  if (this->gicp_dof_mode_ == "planar" && this->track_elevation_enabled_) {
    // Odom init/recovery supplies the map datum once. Subsequent scan solves
    // keep Z fixed and follow only the static TTL elevation profile.
    this->track_z_hold_m_.store(static_cast<double>(p.z()),
                                std::memory_order_relaxed);
    this->track_z_hold_valid_.store(true, std::memory_order_release);
    if (!this->track_output_z_valid_.load(std::memory_order_acquire)) {
      this->track_output_z_m_.store(static_cast<double>(p.z()),
                                    std::memory_order_relaxed);
      this->track_output_z_valid_.store(true, std::memory_order_release);
    }
  }
  this->initialized = true; // publish only after ALL pose state is consistent

  RCLCPP_INFO(this->get_logger(),
              "Received initial pose (%s) at [%.2f, %.2f, %.2f]",
              source.c_str(), p.x(), p.y(), p.z());
}

void gicp_localizer::GicpLocalizer::callbackInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &pose) {

  Eigen::Quaternionf q(
      pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
      pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);

  Eigen::Vector3f p(pose->pose.pose.position.x, pose->pose.pose.position.y,
                    pose->pose.pose.position.z);

  this->applyInitialPose(p, q, pose->header.stamp, "PoseWithCovarianceStamped");
}
