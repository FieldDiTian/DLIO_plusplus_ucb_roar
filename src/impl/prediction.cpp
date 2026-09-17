#include "gicp_interface/detail/localizer_utils.hpp"

void gicp_localizer::GicpLocalizer::callbackImu(
    const sensor_msgs::msg::Imu::SharedPtr imu) {

  // [P1 FIX 2026-07-14] Perform a pending coordinated epoch reset here — at IMU
  // callback entry, with NO estimator locks held (detection in the
  // buffer-insert block below and in callbackGtOdom only SETS the flag; it
  // cannot safely acquire the higher locks while holding a leaf lock).
  // exchange() makes exactly one reentrant IMU thread perform the reset.
  if (this->epoch_reset_pending_.exchange(false)) {
    std::unique_lock<std::mutex> epoch_scan_lock(this->epoch_scan_mtx_);
    this->resetEstimatorForEpochChangeLocked(
        "imu-stream", this->epoch_reset_regress_s_.load());
  }

  double stamp = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;

  Eigen::Vector3f ang_vel(imu->angular_velocity.x, imu->angular_velocity.y,
                          imu->angular_velocity.z);
  Eigen::Vector3f lin_accel(imu->linear_acceleration.x,
                            imu->linear_acceleration.y,
                            imu->linear_acceleration.z);

  // One-shot defensive check: warn if the incoming IMU header.frame_id does
  // not match the configured imu_frame. The single-source P1 design assumes
  // both are "gps_antenna_top"; any other combination usually indicates the
  // imu_topic launch arg was re-pointed at a different IMU (e.g. a NovAtel
  // or VectorNav source) without also updating localization/imu_frame. The
  // code would otherwise silently treat the foreign IMU's axes / lever-arm
  // as if they were at gps_antenna_top, because the TF lookup base_frame ->
  // imu_frame still returns identity in our yaml. Atomic exchange ensures the
  // warning fires exactly once even under the Reentrant callback group.
  // Empty frame_id is tolerated (some drivers leave it unset); only an
  // explicit mismatch trips the warn.
  const bool imu_frame_mismatch =
      !imu->header.frame_id.empty() && imu->header.frame_id != this->imu_frame;
  if (!this->imu_frame_id_checked_.exchange(true)) {
    if (imu_frame_mismatch) {
      RCLCPP_WARN(this->get_logger(),
                  "IMU header.frame_id='%s' does not match configured "
                  "localization/imu_frame='%s'. Treating the IMU axes and "
                  "lever-arm as if at '%s' regardless of the message label. "
                  "If this is intentional (driver mislabels frame_id), "
                  "suppress this warning by updating localization/imu_frame "
                  "to match. If unintentional, the imu_topic remap is "
                  "probably pointing at the wrong IMU.",
                  imu->header.frame_id.c_str(), this->imu_frame.c_str(),
                  this->imu_frame.c_str());
    }
  }
  if (imu_frame_mismatch && this->imu_require_frame_match_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Rejecting IMU sample: header.frame_id='%s' != "
                         "localization/imu_frame='%s'",
                         imu->header.frame_id.c_str(), this->imu_frame.c_str());
    return;
  }

  // Cache IMU-to-baselink transform from TF (once)
  if (!this->imu_extrinsics_cached_) {
    try {
      auto tf_bi = this->tf_buffer->lookupTransform(
          this->base_frame, this->imu_frame, tf2::TimePointZero);
      Eigen::Quaternionf q_bi(
          tf_bi.transform.rotation.w, tf_bi.transform.rotation.x,
          tf_bi.transform.rotation.y, tf_bi.transform.rotation.z);
      Eigen::Vector3f t_bi(tf_bi.transform.translation.x,
                           tf_bi.transform.translation.y,
                           tf_bi.transform.translation.z);
      this->extrinsics.baselink2imu.R = q_bi.toRotationMatrix();
      this->extrinsics.baselink2imu.t = t_bi;
      this->extrinsics.baselink2imu_T.setIdentity();
      this->extrinsics.baselink2imu_T.block<3, 3>(0, 0) =
          q_bi.toRotationMatrix();
      this->extrinsics.baselink2imu_T.block<3, 1>(0, 3) = t_bi;
      this->imu_extrinsics_cached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Cached baselink->imu extrinsic: t=[%.3f,%.3f,%.3f]",
                  t_bi.x(), t_bi.y(), t_bi.z());
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Cannot cache baselink->imu TF: %s", ex.what());
    }
  }

  if (!this->imu_extrinsics_cached_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skipping IMU sample until baselink->imu TF is cached");
    return;
  }

  // Transform IMU measurements from IMU frame to baselink frame.
  const Eigen::Matrix3f &R = this->extrinsics.baselink2imu.R;
  const Eigen::Vector3f &t = this->extrinsics.baselink2imu.t;

  // Rotate angular velocity and linear acceleration to baselink frame
  Eigen::Vector3f ang_vel_bl = R * ang_vel;
  Eigen::Vector3f lin_accel_bl = R * lin_accel;

  // Lever-arm compensation: t is base->IMU, so r_IMU->base = -t.
  // a_base = a_imu - omega x (omega x t) - alpha x t; alpha is approximated
  // as zero because the angular-acceleration term is small at 100 Hz.
  lin_accel_bl -= ang_vel_bl.cross(ang_vel_bl.cross(t));

  ang_vel = ang_vel_bl;
  lin_accel = lin_accel_bl;

  ImuMeas imu_meas_temp;
  imu_meas_temp.stamp = stamp;
  // sensor_msgs/Imu orientation is the IMU-frame attitude in its world frame.
  // Convert it to the configured base frame once at ingestion. A covariance
  // sentinel of -1 means orientation is unavailable and must fail closed.
  const auto &orientation_msg = imu->orientation;
  const double orientation_norm_sq = orientation_msg.w * orientation_msg.w +
                                     orientation_msg.x * orientation_msg.x +
                                     orientation_msg.y * orientation_msg.y +
                                     orientation_msg.z * orientation_msg.z;
  const double yaw_variance = imu->orientation_covariance[8];
  const bool orientation_covariance_available =
      imu->orientation_covariance[0] >= 0.0 && std::isfinite(yaw_variance) &&
      yaw_variance >= 0.0;
  if (orientation_covariance_available && std::isfinite(orientation_norm_sq) &&
      orientation_norm_sq > 1e-12) {
    Eigen::Quaternionf q_world_imu(static_cast<float>(orientation_msg.w),
                                   static_cast<float>(orientation_msg.x),
                                   static_cast<float>(orientation_msg.y),
                                   static_cast<float>(orientation_msg.z));
    q_world_imu.normalize();
    const Eigen::Quaternionf q_imu_base(R.transpose());
    imu_meas_temp.orientation_world_base =
        (q_world_imu * q_imu_base).normalized();
    imu_meas_temp.orientation_yaw_variance = yaw_variance;
    imu_meas_temp.orientation_valid = true;
  }

  // The delayed IESKF consumes the raw, base-frame specific force and angular
  // velocity.  The legacy deskew/observer buffer below remains bias-corrected;
  // feeding that corrected signal to a filter which also owns bias states
  // would subtract calibration twice.
  gicp_localizer::detail::ImuSample fusion_imu_sample;
  fusion_imu_sample.stamp_s = stamp;
  fusion_imu_sample.angular_velocity = ang_vel.cast<double>();
  fusion_imu_sample.linear_acceleration = lin_accel.cast<double>();
  if (imu_meas_temp.orientation_valid) {
    const Eigen::Vector3d orientation_variance(
        imu->orientation_covariance[0], imu->orientation_covariance[4],
        imu->orientation_covariance[8]);
    if (orientation_variance.allFinite() &&
        (orientation_variance.array() >= 0.0).all()) {
      constexpr double kOrientationSigmaFloorRad =
          0.25 * M_PI / 180.0;
      fusion_imu_sample.orientation_world_from_body =
          imu_meas_temp.orientation_world_base.cast<double>();
      fusion_imu_sample.orientation_variance_rad2 =
          orientation_variance.cwiseMax(kOrientationSigmaFloorRad *
                                        kOrientationSigmaFloorRad);
    }
  }
  // P3 (bonus): buffer BIAS-CORRECTED IMU so every consumer — propagateState,
  // integrateImu (T_prior) and the per-point deskew frames — integrates the
  // same corrected signal. Previously only propagateState subtracted state.b;
  // the prior/deskew path integrated the raw gyro/accel, so once RTK
  // calibration set a nonzero bias the two integration paths permanently
  // disagreed (worst during high-yaw-rate sweeps, where deskew rotation error
  // scales directly with the gyro bias). Mirrors upstream DLIO, which applies
  // the bias in the IMU callback before buffering.
  // NOTE: the calibration paths below intentionally keep consuming the RAW
  // ang_vel/lin_accel locals — bias estimation must see the uncorrected
  // signal. state.b is zero until calibration completes, so pre-calibration
  // buffered samples are unaffected.
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    imu_meas_temp.ang_vel = ang_vel - this->state.b.gyro;
    imu_meas_temp.lin_accel = lin_accel - this->state.b.accel;
  }

  // Calculate dt
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      const double newest_stamp = this->imu_buffer.front().stamp;
      // [REVIEW FIX 2026-07-08 P2] Enforce buffer monotonicity. The IMU
      // subscription is in a REENTRANT callback group: two callbacks can
      // finish out of timestamp order, which previously produced a NEGATIVE
      // dt and a non-monotonic buffer — imuMeasFromTimeRange() assumes strict
      // newest-to-oldest order, so a single inversion corrupts propagation,
      // deskew and the yaw priors. Drop the regressed/duplicate sample; at
      // 100+ Hz the information loss is negligible and every downstream
      // invariant holds.
      if (stamp <= newest_stamp) {
        // [P1 FIX 2026-07-14] Distinguish a reentrant-race inversion (ms-scale)
        // from an epoch reset (bag loop / adapter re-anchor / power-cycle,
        // seconds-scale). A large backward jump invalidates the whole estimator
        // timeline, so flag a COORDINATED reset (performed at the next IMU
        // callback entry — see callbackImu top / resetEstimatorForEpochChange).
        // A per-buffer clear here would leave base_pose_stamp_ / deskew /
        // calibration on the old epoch and silently corrupt output. Either way
        // this sample is dropped to keep the buffer monotone.
        if (newest_stamp - stamp > 5.0 && !this->epoch_reset_pending_.load()) {
          this->epoch_reset_regress_s_.store(newest_stamp - stamp);
          this->epoch_reset_pending_.store(true);
          RCLCPP_WARN(this->get_logger(),
                      "IMU stamp EPOCH RESET detected (rewind %.3f s → %.6f); "
                      "scheduling coordinated "
                      "estimator re-initialization.",
                      newest_stamp - stamp, stamp);
        } else {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "IMU sample out of order (%.6f <= newest %.6f) — dropped to keep "
              "the buffer monotonic (reentrant callback race)",
              stamp, newest_stamp);
        }
        return;
      }
      imu_meas_temp.dt = stamp - newest_stamp;
    } else {
      imu_meas_temp.dt = 0.0;
    }

    this->imu_buffer.push_front(imu_meas_temp);
    this->imu_meas = imu_meas_temp;
  }
  this->imu_cv_.notify_all();

  if (!this->first_imu_received) {
    this->first_imu_received = true;
    this->first_imu_stamp_ = stamp;
    RCLCPP_INFO(this->get_logger(), "First IMU message received");

    // If RTK init is off entirely, go straight to the stationary path.
    if (!this->rtk_init_enabled_ &&
        this->init_phase_.load() == InitPhase::WAITING) {
      this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    }
  }

  // Calibration phase machine. We may be:
  //   WAITING                 — RTK init enabled but no GT received yet
  //   RTK_CALIBRATING         — first GT arrived; accumulating IMU residuals
  //   against GT truth STATIONARY_CALIBRATING  — fallback (no GT in time, or
  //   RTK init disabled) DONE                    — biases applied; propagate
  //   normally
  // ----- Initialization / IMU-bias calibration (seed-and-go) -----
  // Localization output must never stop. We DECOUPLE output from bias
  // calibration: as soon as a usable state seed exists (initialized with a
  // pose/orientation from GT odom-init, a param initial pose, or the first
  // GICP scan), we fall through to propagateState() below and publish
  // continuously, using the most stable information available at the time.
  // Bias calibration (RTK-driven or stationary) then runs in the BACKGROUND
  // and applies its refined bias as a smooth correction when it completes --
  // no output gap. We suppress output (early return) ONLY while there is
  // genuinely nothing to propagate from yet, i.e. before any seed exists; in
  // that unseeded window the original WAITING -> RTK/STATIONARY fallback logic
  // still applies so we reach a first fix as fast as possible.
  if (!this->imu_calibrated_) {
    // [REVIEW FIX 2026-07-08 P2] Serialize the init/bias-calibration state
    // machine. The IMU subscription is in a REENTRANT callback group under a
    // MultiThreadedExecutor: without this lock, parallel IMU callbacks could
    // interleave in the WAITING/RTK/STATIONARY phase transitions, corrupt the
    // RTK/stationary accumulator sums, or double-finalize the calibration.
    // Scope: startup only — once imu_calibrated_ is true the outer atomic
    // check bypasses the lock forever.
    std::lock_guard<std::mutex> calib_lock(this->calib_mtx_);
    if (this->imu_calibrated_) {
      // Another IMU thread finalized calibration while we waited on the
      // lock; fall through to normal propagation below.
    } else {

      InitPhase phase = this->init_phase_.load();

      // True once we have something stable to propagate from. Mirrors the
      // propagateState() gate below so that "fall through" always yields
      // output.
      const bool can_propagate_now =
          this->initialized.load() &&
          (this->geo.first_opt_done.load() || this->imu_only_mode_);

      if (phase == InitPhase::WAITING) {
        if (this->gt_odom_received_.load()) {
          // First GT sample has arrived — start RTK-driven calibration on the
          // next IMU.
          this->init_phase_ = InitPhase::RTK_CALIBRATING;
          this->rtk_calib_start_stamp_ = stamp;
          RCLCPP_INFO(
              this->get_logger(),
              "RTK init: GT odom received; starting RTK-driven IMU calibration "
              "(window=%.1fs)",
              this->rtk_calib_window_sec_);
          phase = InitPhase::RTK_CALIBRATING;
        } else if (stamp - this->first_imu_stamp_ >
                   this->rtk_fallback_timeout_sec_) {
          // No GT in time — fall back to stationary calibration.
          RCLCPP_WARN(
              this->get_logger(),
              "RTK init: no GT odom within %.1fs of first IMU; falling back to "
              "stationary IMU calibration",
              this->rtk_fallback_timeout_sec_);
          this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
          this->imu_calib_start_stamp_ =
              stamp; // reset so the existing window starts now
          phase = InitPhase::STATIONARY_CALIBRATING;
        } else {
          RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "RTK init: waiting for first GT odom (elapsed=%.1f/%.1fs)",
              stamp - this->first_imu_stamp_, this->rtk_fallback_timeout_sec_);
          // Seed-and-go: only block output if we have no seed yet. With a param
          // seed we dead-reckon here until GT arrives instead of going dark.
          if (!can_propagate_now)
            return;
        }
      }

      if (phase == InitPhase::RTK_CALIBRATING) {
        // Accumulate an RTK-driven bias estimate in the background. It only
        // completes on RTK-FIXED pairings; with degraded-only Atlas odom it
        // never finishes -- but that is now BENIGN, because once a seed exists
        // we keep propagating below rather than stalling, and a later FIXED
        // sample still completes the calibration opportunistically. We only
        // force a stationary fallback while still UNSEEDED (output dark), so a
        // usable estimate is reached promptly in that case (the P1 stall fix).
        if (!can_propagate_now) {
          const double rtk_elapsed = stamp - this->rtk_calib_start_stamp_;
          const double rtk_phase_budget =
              this->rtk_calib_window_sec_ + this->rtk_fallback_timeout_sec_;
          if (rtk_elapsed > rtk_phase_budget) {
            RCLCPP_WARN(
                this->get_logger(),
                "RTK init: %.1fs in RTK_CALIBRATING with no seed and no usable "
                "RTK-FIXED GT pairing; falling back to stationary IMU "
                "calibration",
                rtk_elapsed);
            this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
            this->imu_calib_start_stamp_ =
                stamp; // restart window in the stationary path
            return; // still unseeded — stationary calibration begins next IMU
          }
        }
        if (this->tryRtkCalibrationStep(stamp, ang_vel, lin_accel)) {
          // tryRtkCalibrationStep applied biases + seeded state and set
          // imu_calibrated_.
          this->init_phase_ = InitPhase::DONE;
        } else if (!this->imu_calibrated_ && !can_propagate_now) {
          return; // no seed yet — keep output suppressed until we have one
        }
        // Seeded: fall through and propagate; RTK bias refines
        // opportunistically.
      } else if (phase == InitPhase::STATIONARY_CALIBRATING) {
        // Stationary path: assume omega=0 and accel direction = gravity. Used
        // when no GT seed is available; it provides the initial
        // gravity/orientation.
        if (this->imu_calib_start_stamp_ < 0.0) {
          this->imu_calib_start_stamp_ = stamp;
        }

        this->imu_calib_gyro_sum_ += ang_vel;
        this->imu_calib_accel_sum_ += lin_accel;
        this->imu_calib_gyro_sq_sum_ += ang_vel.cwiseProduct(ang_vel);
        this->imu_calib_count_++;

        double elapsed = stamp - this->imu_calib_start_stamp_;
        if (elapsed >= this->imu_calib_time_ && this->imu_calib_count_ > 0) {
          const float calib_n = static_cast<float>(this->imu_calib_count_);
          Eigen::Vector3f gyro_avg = this->imu_calib_gyro_sum_ / calib_n;
          Eigen::Vector3f accel_avg = this->imu_calib_accel_sum_ / calib_n;

          // [P3 FIX 2026-07-14] Stationarity sanity check. This path ASSUMES
          // omega=0; under seed-and-go with GNSS absent the window can complete
          // while the vehicle is actually turning (up to ~0.5 rad/s), which
          // would bake real body rates permanently into state.b.gyro. Require
          // the mean gyro near zero AND small per-axis spread AND |accel| ~
          // gravity, plus the RTK path's plausibility bounds. If the window is
          // not stationary, restart it rather than trusting a
          // motion-contaminated bias — the gyro bias stays at its safe default
          // (0) until a stationary window or an opportunistic RTK-FIXED pairing
          // calibrates it.
          constexpr float kMaxStationaryGyroMean = 0.05f; // ~3 deg/s
          constexpr float kMaxStationaryGyroStd = 0.05f;
          const Eigen::Vector3f gyro_var =
              (this->imu_calib_gyro_sq_sum_ / calib_n -
               gyro_avg.cwiseProduct(gyro_avg))
                  .cwiseMax(0.0f);
          const float gyro_std_max = std::sqrt(gyro_var.maxCoeff());
          const float g = static_cast<float>(this->gravity_);
          const bool stationary =
              gyro_avg.norm() < kMaxStationaryGyroMean &&
              gyro_std_max < kMaxStationaryGyroStd &&
              gyro_avg.norm() < 1.0f && // RTK-path plausibility bound
              std::abs(accel_avg.norm() - g) <
                  0.5f * g; // |accel| ~ gravity, not accelerating
          if (!stationary) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Stationary IMU calibration: window NOT "
                                 "stationary (|gyro_mean|=%.3f rad/s, "
                                 "gyro_std=%.3f rad/s, |accel|=%.2f m/s^2); "
                                 "vehicle likely moving. Restarting the "
                                 "window; gyro bias stays 0 until a stationary "
                                 "window or an RTK-FIXED pairing.",
                                 gyro_avg.norm(), gyro_std_max,
                                 accel_avg.norm());
            this->imu_calib_start_stamp_ = stamp;
            this->imu_calib_count_ = 0;
            this->imu_calib_gyro_sum_.setZero();
            this->imu_calib_accel_sum_.setZero();
            this->imu_calib_gyro_sq_sum_.setZero();
            if (!can_propagate_now)
              return; // unseeded: keep waiting for a stationary window
            // seeded: fall through past the bake and keep propagating; the
            // window retries in the background.
          } else {
            // [P2 FIX 2026-07-09] Gravity SIGN. This rig's documented
            // convention (see the specific-force note in tryRtkCalibrationStep)
            // is that a level stationary body reads accel ~= (0,0,+g), and the
            // propagation math (integrateImu / propagateState) computes
            // world_accel = q*accel then subtracts +g on world Z — which
            // requires q_init to map the measured accel direction onto world
            // +Z. The previous (0,0,-1) target was a pi flip:
            // FromTwoVectors(+z, -z) started the observer upside-down with
            // arbitrary yaw; the bias math was self-consistent with the flip
            // (so nothing caught it), propagateState produced -2g vertical
            // acceleration, and the delta-form updateState never corrects
            // absolute attitude, so the IMU-rate output stayed flipped until a
            // GT snap. Both the target vector and expected_grav_body flip
            // TOGETHER so the bias estimate stays correct.
            Eigen::Vector3f grav_world(0.f, 0.f, +1.f);
            Eigen::Vector3f grav_body = accel_avg.normalized();
            Eigen::Quaternionf q_init =
                Eigen::Quaternionf::FromTwoVectors(grav_body, grav_world);

            Eigen::Vector3f expected_grav_body =
                q_init.conjugate()._transformVector(Eigen::Vector3f(
                    0.f, 0.f, +static_cast<float>(this->gravity_)));

            // [REVIEW FIX 2026-07-08 P3] ALL observer-state writes under
            // geo.mtx. The IMU subscription runs in a REENTRANT callback group,
            // so parallel IMU callbacks can be inside propagateState()/bias
            // reads while this branch fires — the previous unlocked
            // state.b.gyro / state.b.accel stores were a real data race.
            {
              std::lock_guard<std::mutex> lock(this->geo.mtx);
              this->state.b.gyro = gyro_avg;
              this->state.b.accel = accel_avg - expected_grav_body;
              // Only adopt the accel-derived orientation when we have no better
              // one. [P2 FIX 2026-07-09] "Better one" must cover ALL seed
              // sources: the old guard checked only the param-pose and GT
              // odom-init flags, so an /initialpose (RViz) seed — which sets
              // `initialized` and possibly `first_opt_done` but neither flag —
              // was clobbered when the stationary window completed seconds
              // later, permanently (the delta-form observer never restores
              // absolute attitude, and first_opt_done being true blocks the
              // first-scan re-init). Overwriting a live estimate also violates
              // the stationary assumption if the vehicle has started moving.
              if (!this->use_param_initial_pose_ &&
                  !this->use_odom_init_applied_ && !this->initialized.load() &&
                  !this->geo.first_opt_done.load()) {
                this->state.q = q_init;
                this->geo.prev_q = q_init;
              }
              ++this->geo.update_seq; // discard in-flight propagateState
                                      // computations
            }

            this->imu_calibrated_ = true;
            this->init_phase_ = InitPhase::DONE;
            RCLCPP_INFO(
                this->get_logger(),
                "IMU calibrated (stationary, %d samples, %.1fs): "
                "gyro_bias=[%.4f,%.4f,%.4f] "
                "accel_bias=[%.3f,%.3f,%.3f] gravity_dir=[%.3f,%.3f,%.3f]",
                this->imu_calib_count_, elapsed, gyro_avg.x(), gyro_avg.y(),
                gyro_avg.z(), this->state.b.accel.x(), this->state.b.accel.y(),
                this->state.b.accel.z(), grav_body.x(), grav_body.y(),
                grav_body.z());
          } // end else (stationary window accepted)
        } else if (!can_propagate_now) {
          RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "IMU calibrating (stationary)... %.1f/%.1fs (%d samples)",
              elapsed, this->imu_calib_time_, this->imu_calib_count_);
          return; // no seed yet — suppress output until gravity/orientation
                  // known
        }
        // Seeded: fall through and propagate while stationary stats accumulate.
      }
    }
  }

  // Keep the geometric observer current at the native IMU rate. This is an
  // INTERNAL prediction/recovery state; normal GICP mode still publishes only
  // accepted scan-rate solutions from publishPose(). Disabling high-rate
  // product output must never disable high-rate inertial propagation.
  // Note: function-local statics here and in propagateState() (skip_count,
  // propagate_count, logged_first_publish, path_decimator, odom_publish_count,
  // last_report_time) are safe ONLY because the IMU callback group is
  // MutuallyExclusive (see its creation) — no two IMU callbacks run
  // concurrently. thread_local was wrong under exclusivity too: exclusive
  // callbacks can hop threads, splitting the counts.
  static int propagate_calls = 0;
  static int imu_total = 0;
  static bool logged_first_propagate = false;
  imu_total++;

  if (this->initialized &&
      (this->geo.first_opt_done || this->imu_only_mode_)) {
    this->propagateState(imu_meas_temp);
    this->updateDelayedFusionWithImu(fusion_imu_sample);
    propagate_calls++;

    // Log first successful propagation
    if (!logged_first_propagate) {
      RCLCPP_INFO(this->get_logger(),
                  "First internal IMU propagation successful");
      logged_first_propagate = true;
    }
  }

  // This is an opt-in development metric.  The IMU callback runs at sensor
  // rate, so even a once-per-100-callback status line is terminal churn in a
  // deployment profile.
  static int imu_count = 0;
  if (this->verbose_ && ++imu_count % 100 == 0) {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    RCLCPP_INFO(this->get_logger(),
                "IMU rate check: %d callbacks, %d propagations, "
                "initialized=%d, geo_init=%d",
                imu_total, propagate_calls, this->initialized.load(),
                this->geo.first_opt_done.load());
    imu_total = 0;
    propagate_calls = 0;
  }
}

bool gicp_localizer::GicpLocalizer::imuMeasFromTimeRange(
    double start_time, double end_time, std::vector<ImuMeas> &out) {

  std::lock_guard<std::mutex> lock(this->mtx_imu);

  constexpr double kOlderToleranceSec = 0.002;
  if (!selectBracketedImuRange(this->imu_buffer, start_time, end_time,
                               kOlderToleranceSec, out)) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU range unavailable: request=[%.6f,%.6f] buffer=[oldest=%.6f,"
        "newest=%.6f,size=%zu]",
        start_time, end_time,
        this->imu_buffer.empty() ? -1.0 : this->imu_buffer.back().stamp,
        this->imu_buffer.empty() ? -1.0 : this->imu_buffer.front().stamp,
        this->imu_buffer.size());
    return false;
  }
  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_localizer::GicpLocalizer::integrateImu(
    double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
    Eigen::Vector3f v_init, const std::vector<double> &sorted_timestamps,
    std::vector<Eigen::Vector3f> *velocities_out) {

  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      empty;
  if (velocities_out)
    velocities_out->clear();

  if (sorted_timestamps.empty()) {
    if (this->verbose_) {
      std::fprintf(stderr, "[IMU_INT] REJECT guard: empty timestamps\n");
      std::fflush(stderr);
    }
    return empty;
  }
  // [P1 FIX 2026-07-10] Overlapping merged sweeps are NORMAL: a leading aux
  // LiDAR contributes points earlier than the seed time (with the seed now at
  // the previous MEDIAN time, current-scan aux points routinely precede it at
  // 20 Hz). The old `start_time > timestamps.front()` rejection threw away
  // the ENTIRE frame's deskew + motion prediction exactly on those frames.
  // Instead: extend the IMU slice down to the earliest point and let the
  // existing seed back-projection (idt = start_time - f1.stamp) handle the
  // offset. A gross inconsistency still fails closed via the bound below.
  constexpr double kMaxSeedLeadSec = 0.5;
  if (start_time - sorted_timestamps.front() > kMaxSeedLeadSec) {
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT guard: seed leads earliest point by %.3fs "
                   "(> %.1fs) — "
                   "inconsistent merge\n",
                   start_time - sorted_timestamps.front(), kMaxSeedLeadSec);
      std::fflush(stderr);
    }
    return empty;
  }
  const double slice_start = std::min(start_time, sorted_timestamps.front());

  std::vector<ImuMeas> imu_slice;
  if (this->imuMeasFromTimeRange(slice_start, sorted_timestamps.back(),
                                 imu_slice) == false) {
    double front_s = -1, back_s = -1;
    size_t sz = 0;
    {
      std::lock_guard<std::mutex> lk(this->mtx_imu);
      sz = this->imu_buffer.size();
      if (sz > 0) {
        front_s = this->imu_buffer.front().stamp;
        back_s = this->imu_buffer.back().stamp;
      }
    }
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT range: start=%.6f end=%.6f buf_sz=%zu "
                   "front=%.6f back=%.6f (front<end=%d)\n",
                   start_time, sorted_timestamps.back(), sz, front_s, back_s,
                   (int)(sz > 0 && front_s < sorted_timestamps.back()));
      std::fflush(stderr);
    }
    return empty;
  }

  if (imu_slice.size() < 2) {
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT slice<2: start=%.6f end=%.6f "
                   "first.stamp=%.6f last.stamp=%.6f\n",
                   start_time, sorted_timestamps.back(),
                   imu_slice.empty() ? 0.0 : imu_slice.front().stamp,
                   imu_slice.empty() ? 0.0 : imu_slice.back().stamp);
      std::fflush(stderr);
    }
    return empty;
  }

  const ImuMeas &f1 = imu_slice[0];
  const ImuMeas &f2 = imu_slice[1];

  // Time between first two IMU samples
  double dt = f2.dt;

  if (dt < 1e-6) {
    if (this->verbose_) {
      std::fprintf(
          stderr,
          "[IMU_INT] REJECT dt: f1.stamp=%.6f f2.stamp=%.6f f2.dt=%.9f\n",
          f1.stamp, f2.stamp, dt);
      std::fflush(stderr);
    }
    return empty;
  }

  // [P2 FIX 2026-07-10c] Piecewise REVERSE integration of the seed from
  // start_time back to the slice head, through the REAL IMU samples. The
  // previous code extrapolated the FIRST interval's angular acceleration and
  // jerk across the entire seed lead (up to kMaxSeedLeadSec after the
  // overlapping-sweep fix): under changing dynamics the forward pass through
  // the real samples then did not return to the supplied seed pose at
  // start_time, with error growing with overlap duration. Each reverse step
  // below is the algebraic inverse of the forward scheme in
  // integrateImuInternal (same first-order quaternion update, same
  // trapezoidal velocity and jerk-aware position updates), so the forward
  // trajectory re-anchors at the seed to the scheme's own accuracy.
  auto advance_q = [](const Eigen::Quaternionf &qq, const Eigen::Vector3f &w,
                      float h) {
    Eigen::Quaternionf r(
        qq.w() - 0.5f * (qq.x() * w[0] + qq.y() * w[1] + qq.z() * w[2]) * h,
        qq.x() + 0.5f * (qq.w() * w[0] - qq.z() * w[1] + qq.y() * w[2]) * h,
        qq.y() + 0.5f * (qq.z() * w[0] + qq.w() * w[1] - qq.x() * w[2]) * h,
        qq.z() + 0.5f * (qq.x() * w[1] - qq.y() * w[0] + qq.w() * w[2]) * h);
    r.normalize();
    return r;
  };
  auto world_accel = [this](const Eigen::Quaternionf &qq,
                            const Eigen::Vector3f &acc) {
    Eigen::Vector3f wa = qq._transformVector(acc);
    wa[2] -= static_cast<float>(this->gravity_);
    return wa;
  };

  // Index of the last sample at/before start_time.
  // [SELF-AUDIT FIX 2026-07-10] The earlier claim "m + 1 is always valid"
  // assumed start_time < end_time — FALSE on the deskew-off branch, where
  // single_ts[0] is clamped to latest_imu: with a lagging/stalled IMU stream
  // the seed time (previous median) can EXCEED every slice stamp, the loop
  // drives m to size-1, and imu_slice[m + 1] read out of bounds. Guard both
  // ways: reject when the seed is beyond the IMU horizon by more than a
  // fraction of a scan period (too stale to place the seed honestly), else
  // clamp m so the partial step extrapolates on the LAST real interval
  // (idt > dt_int is fine — the formulas are polynomial in idt).
  if (start_time - imu_slice.back().stamp > 0.2) {
    if (this->verbose_) {
      std::fprintf(stderr,
                   "[IMU_INT] REJECT stale: seed %.6f is %.3fs beyond newest "
                   "usable IMU %.6f\n",
                   start_time, start_time - imu_slice.back().stamp,
                   imu_slice.back().stamp);
      std::fflush(stderr);
    }
    return empty;
  }
  size_t m = 0;
  while (m + 1 < imu_slice.size() && imu_slice[m + 1].stamp <= start_time)
    m++;
  if (m + 1 >= imu_slice.size()) {
    m = imu_slice.size() -
        2; // size >= 2 guaranteed by the slice<2 reject above
  }

  // Partial step: start_time -> imu_slice[m].stamp with interval (m, m+1)
  // dynamics (exact inverse of the forward interpolation formulas).
  {
    const ImuMeas &lo = imu_slice[m];
    const ImuMeas &hi = imu_slice[m + 1];
    const double dt_int = hi.stamp - lo.stamp;
    const double idt = start_time - lo.stamp;
    if (dt_int >= 1e-6 && idt > 0.0) {
      const float fdt = static_cast<float>(dt_int);
      const float fidt = static_cast<float>(idt);
      const Eigen::Vector3f alpha = (hi.ang_vel - lo.ang_vel) / fdt;
      const Eigen::Vector3f omega_i = lo.ang_vel + 0.5f * alpha * fidt;
      q_init = advance_q(q_init, omega_i, -fidt); // attitude at lo
      const Eigen::Vector3f a_lo = world_accel(q_init, lo.lin_accel);
      const Eigen::Quaternionf q_hi =
          advance_q(q_init, lo.ang_vel + 0.5f * (hi.ang_vel - lo.ang_vel), fdt);
      const Eigen::Vector3f a_hi = world_accel(q_hi, hi.lin_accel);
      const Eigen::Vector3f jj = (a_hi - a_lo) / fdt;
      v_init -= a_lo * fidt + 0.5f * jj * fidt * fidt;
      p_init -= v_init * fidt + 0.5f * a_lo * fidt * fidt +
                (1.0f / 6.0f) * jj * fidt * fidt * fidt;
    }
  }

  // Full intervals: state at imu_slice[j] -> state at imu_slice[j-1], each
  // inverted with ITS OWN interval's measurements (not an extrapolation).
  for (size_t j = m; j >= 1; j--) {
    const ImuMeas &lo = imu_slice[j - 1];
    const ImuMeas &hi = imu_slice[j];
    const float h = static_cast<float>(hi.stamp - lo.stamp);
    if (h < 1e-6f)
      continue;
    const Eigen::Vector3f omega_avg =
        lo.ang_vel + 0.5f * (hi.ang_vel - lo.ang_vel);
    const Eigen::Vector3f a_hi =
        world_accel(q_init, hi.lin_accel);     // attitude still at hi
    q_init = advance_q(q_init, omega_avg, -h); // attitude at lo
    const Eigen::Vector3f a_lo = world_accel(q_init, lo.lin_accel);
    // Forward: v_hi = v_lo + 0.5*(a_lo + a_hi)*h
    //          p_hi = p_lo + v_lo*h + 0.5*a_lo*h^2 + (1/6)*(a_hi - a_lo)*h^2
    v_init -= 0.5f * (a_lo + a_hi) * h;
    p_init -= v_init * h + 0.5f * a_lo * h * h +
              (1.0f / 6.0f) * (a_hi - a_lo) * h * h;
  }

  return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps,
                                    imu_slice, velocities_out);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_localizer::GicpLocalizer::integrateImuInternal(
    Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double> &sorted_timestamps,
    const std::vector<ImuMeas> &imu_slice,
    std::vector<Eigen::Vector3f> *velocities_out) {

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      imu_se3;
  if (velocities_out) {
    velocities_out->clear();
    velocities_out->reserve(sorted_timestamps.size());
  }

  // Initialization
  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  Eigen::Vector3f a = q._transformVector(imu_slice.front().lin_accel);
  a[2] -= this->gravity_;

  // Iterate over IMU measurements and timestamps (imu_slice is a private
  // copy in forward time order — no shared-buffer iterators, see P1 fix)
  auto prev_imu_it = imu_slice.begin();
  auto imu_it = prev_imu_it + 1;

  auto stamp_it = sorted_timestamps.begin();

  for (; imu_it != imu_slice.end(); imu_it++) {

    const ImuMeas &f0 = *prev_imu_it;
    const ImuMeas &f = *imu_it;

    // Time between IMU samples
    double dt = f.dt;

    if (dt < 1e-6) {
      prev_imu_it = imu_it;
      continue;
    }

    // Angular acceleration
    Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
    Eigen::Vector3f alpha = alpha_dt / dt;

    // Average angular velocity
    Eigen::Vector3f omega = f0.ang_vel + 0.5 * alpha_dt;

    // [P1 FIX 2026-07-10] Save the orientation AT f0 before advancing: the
    // interpolation loop below produces poses for timestamps inside [f0, f]
    // with idt measured FROM f0, so it must start from q(f0). The original
    // (upstream-DLIO-inherited) code advanced q to f first and then built
    // q_i on the advanced value — every deskew pose was one full IMU
    // interval ahead in orientation (~10 ms at 100 Hz ~= 0.3 deg at
    // 30 deg/s), while its position interpolated consistently from f0.
    const Eigen::Quaternionf q0 = q;

    // Orientation (advance f0 -> f; used for the acceleration at f and as
    // the next interval's base)
    q = Eigen::Quaternionf(
        q.w() -
            0.5 * (q.x() * omega[0] + q.y() * omega[1] + q.z() * omega[2]) * dt,
        q.x() +
            0.5 * (q.w() * omega[0] - q.z() * omega[1] + q.y() * omega[2]) * dt,
        q.y() +
            0.5 * (q.z() * omega[0] + q.w() * omega[1] - q.x() * omega[2]) * dt,
        q.z() + 0.5 * (q.x() * omega[1] - q.y() * omega[0] + q.w() * omega[2]) *
                    dt);
    q.normalize();

    // Acceleration
    Eigen::Vector3f a0 = a;
    a = q._transformVector(f.lin_accel);
    a[2] -= this->gravity_;

    // Jerk
    Eigen::Vector3f j_dt = a - a0;
    Eigen::Vector3f j = j_dt / dt;

    // Interpolate for given timestamps
    while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
      // Time between previous IMU sample and given timestamp
      double idt = *stamp_it - f0.stamp;

      // Average angular velocity
      Eigen::Vector3f omega_i = f0.ang_vel + 0.5 * alpha * idt;

      // Orientation — from q0 (the orientation at f0), matching idt's origin.
      Eigen::Quaternionf q_i(
          q0.w() - 0.5 *
                       (q0.x() * omega_i[0] + q0.y() * omega_i[1] +
                        q0.z() * omega_i[2]) *
                       idt,
          q0.x() + 0.5 *
                       (q0.w() * omega_i[0] - q0.z() * omega_i[1] +
                        q0.y() * omega_i[2]) *
                       idt,
          q0.y() + 0.5 *
                       (q0.z() * omega_i[0] + q0.w() * omega_i[1] -
                        q0.x() * omega_i[2]) *
                       idt,
          q0.z() + 0.5 *
                       (q0.x() * omega_i[1] - q0.y() * omega_i[0] +
                        q0.w() * omega_i[2]) *
                       idt);
      q_i.normalize();

      // Position
      Eigen::Vector3f p_i =
          p + v * idt + 0.5 * a0 * idt * idt + (1 / 6.) * j * idt * idt * idt;
      const Eigen::Vector3f v_i = v + a0 * idt + 0.5 * j * idt * idt;

      // Transformation
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
      T.block(0, 3, 3, 1) = p_i;

      imu_se3.push_back(T);
      if (velocities_out)
        velocities_out->push_back(v_i);

      stamp_it++;
    }

    // Position
    p += v * dt + 0.5 * a0 * dt * dt + (1 / 6.) * j_dt * dt * dt;

    // Velocity
    v += a0 * dt + 0.5 * j_dt * dt;

    prev_imu_it = imu_it;
  }

  return imu_se3;
}

void gicp_localizer::GicpLocalizer::propagateState(const ImuMeas &imu_local) {

  // [REVIEW FIX 2026-07-08 P1] The caller's OWN sample is propagated, not the
  // shared this->imu_meas latest — under the old Reentrant group, reading the
  // global latest meant an older callback could propagate a newer sample or
  // two callbacks could propagate the same one. With the now-exclusive IMU
  // group this is belt-and-braces, but it also makes the data flow exact:
  // each sample is propagated exactly once, in arrival order.

  double dt = imu_local.dt;

  if (dt <= 0.0 || dt > 1.0) {
    static int skip_count = 0;
    if (++skip_count % 100 == 0) {
      RCLCPP_WARN(
          this->get_logger(),
          "Skipping propagation due to invalid dt: %.6f (skipped %d times)", dt,
          skip_count);
    }
    return; // Skip invalid dt
  }

  // Read current state with minimal lock time
  Eigen::Vector3f current_p;
  Eigen::Quaternionf current_q;
  Eigen::Vector3f current_v_lin_w;
  Eigen::Vector3f bias_gyro;
  Eigen::Vector3f bias_accel;
  uint64_t seq_at_read;

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    current_p = this->state.p;
    current_q = this->state.q;
    current_v_lin_w = this->state.v.lin.w;
    bias_gyro = this->state.b.gyro;
    bias_accel = this->state.b.accel;
    seq_at_read = this->geo.update_seq;
  }

  // Do computation without holding lock
  Eigen::Quaternionf qhat = current_q;
  Eigen::Quaternionf omega;
  Eigen::Vector3f world_accel;

  // P3: biases are now subtracted ONCE, at buffering time in callbackImu, so
  // the buffered measurement is already corrected — do NOT subtract again here
  // (bias_gyro/bias_accel are still read above for the periodic status log).
  Eigen::Vector3f ang_vel_corrected = imu_local.ang_vel;
  Eigen::Vector3f lin_accel_corrected = imu_local.lin_accel;

  // Transform accel from body to world frame and subtract gravity
  world_accel = qhat._transformVector(lin_accel_corrected);

  // Keep observer-state dumps out of the normal node terminal.  Diagnostics
  // and invalid-state warnings below remain enabled independently.
  static int propagate_count = 0;
  if (this->verbose_ && ++propagate_count % 1000 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Geo Observer: pos_z=%.3f vel_z=%.3f | accel_raw_z=%.3f "
                "bias_z=%.3f world_accel_z=%.3f | gravity=%.2f",
                current_p.z(), current_v_lin_w.z(), imu_local.lin_accel.z(),
                bias_accel.z(), world_accel.z(), this->gravity_);
  }

  // Position propagation (with gravity compensation)
  Eigen::Vector3f new_p = current_p;
  new_p += current_v_lin_w * dt + 0.5f * dt * dt * world_accel;
  new_p[2] -= 0.5f * dt * dt * static_cast<float>(this->gravity_);

  // Velocity propagation (with gravity compensation)
  Eigen::Vector3f new_v_lin_w = current_v_lin_w + world_accel * dt;
  new_v_lin_w[2] -= dt * static_cast<float>(this->gravity_);

  // Ground vehicle Z-velocity damping (same as in updateState)
  new_v_lin_w[2] *= (1.0f - dt * static_cast<float>(this->geo_Kz_damping_));
  const bool planar_z_locked =
      this->gicp_dof_mode_ == "planar" && this->track_elevation_enabled_ &&
      this->track_z_hold_valid_.load(std::memory_order_acquire);
  if (planar_z_locked) {
    // Z is a static-map quantity in planar mode. Do not let accelerometer
    // bias/gravity error re-introduce vertical motion between scan updates.
    new_p.z() = static_cast<float>(
        this->track_z_hold_m_.load(std::memory_order_relaxed));
    new_v_lin_w.z() = 0.0f;
  }
  if (this->geo_max_state_speed_ > 0.0) {
    const float speed = new_v_lin_w.norm();
    if (std::isfinite(speed) &&
        speed > static_cast<float>(this->geo_max_state_speed_)) {
      new_v_lin_w *= static_cast<float>(this->geo_max_state_speed_) / speed;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "propagateState: observer speed %.1fm/s exceeded "
                           "physical cap %.1fm/s; "
                           "clamped (inspect GICP rejection/recovery)",
                           static_cast<double>(speed),
                           this->geo_max_state_speed_);
    }
  }

  // Orientation propagation
  omega.w() = 0;
  omega.vec() = ang_vel_corrected;
  Eigen::Quaternionf tmp = qhat * omega;
  Eigen::Quaternionf new_q;
  new_q.w() = qhat.w() + 0.5 * dt * tmp.w();
  new_q.vec() = qhat.vec() + 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  new_q.normalize();

  // Store angular velocity
  Eigen::Vector3f new_v_ang_b = ang_vel_corrected;
  Eigen::Vector3f new_v_ang_w = new_q.toRotationMatrix() * new_v_ang_b;

  // Validate computed state before publishing
  bool state_valid =
      std::isfinite(new_p.x()) && std::isfinite(new_p.y()) &&
      std::isfinite(new_p.z()) && std::isfinite(new_q.w()) &&
      std::isfinite(new_q.x()) && std::isfinite(new_q.y()) &&
      std::isfinite(new_q.z()) && std::isfinite(new_v_lin_w.x()) &&
      std::isfinite(new_v_lin_w.y()) && std::isfinite(new_v_lin_w.z());

  if (!state_valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Skipping odometry publish - state contains invalid "
                         "values (p=[%.3f,%.3f,%.3f], q=[%.3f,%.3f,%.3f,%.3f])",
                         new_p.x(), new_p.y(), new_p.z(), new_q.w(), new_q.x(),
                         new_q.y(), new_q.z());
    return; // Skip publishing if state contains invalid values
  }

  // Commit the internal observer before considering output. If updateState()
  // corrected the state while this IMU sample was being integrated, discard
  // this stale propagation instead of overwriting the new GICP anchor.
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    if (this->geo.update_seq == seq_at_read) {
      this->state.p = new_p;
      this->state.q = new_q;
      this->state.v.lin.w = new_v_lin_w;
      this->state.v.lin.b = new_q.conjugate() * new_v_lin_w;
      this->state.v.ang.b = new_v_ang_b;
      this->state.v.ang.w = new_v_ang_w;
      this->observer_pose_history_.push_back(
          {imu_local.stamp, new_p, new_q, this->observer_epoch_});
      while (this->observer_pose_history_.size() > this->imu_buffer_size_) {
        this->observer_pose_history_.pop_front();
      }
    }
  }

  // In normal localization mode propagation is compute-only. Product pose,
  // odom and TF are emitted together by publishPose() after an accepted GICP
  // solve, so their rate remains the configured scan rate (4 Hz here).
  if (!this->imu_only_mode_) {
    return;
  }

  // Use IMU timestamp (from bag file or sensor)
  rclcpp::Time current_time;
  current_time = rclcpp::Time(static_cast<int64_t>(imu_local.stamp * 1e9));

  Eigen::Vector3f output_p = new_p;
  const bool planar_output_z_locked =
      this->gicp_dof_mode_ == "planar" && this->track_elevation_enabled_ &&
      this->track_output_z_valid_.load(std::memory_order_acquire);
  if (planar_output_z_locked) {
    output_p.z() = static_cast<float>(
        this->track_output_z_m_.load(std::memory_order_relaxed));
  }

  // Log successful validation on first publish
  static bool logged_first_publish = false;
  if (!logged_first_publish) {
    RCLCPP_INFO(this->get_logger(),
                "First odometry publish! Using IMU timestamp: %.3f",
                imu_local.stamp);
    logged_first_publish = true;
  }

  // Build odometry message from computed values (not from this->state to avoid
  // race condition)
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = this->map_frame;
  odom_msg.child_frame_id = this->base_frame;

  // Position and orientation from propagated state
  odom_msg.pose.pose.position.x = output_p.x();
  odom_msg.pose.pose.position.y = output_p.y();
  odom_msg.pose.pose.position.z = output_p.z();
  odom_msg.pose.pose.orientation.w = new_q.w();
  odom_msg.pose.pose.orientation.x = new_q.x();
  odom_msg.pose.pose.orientation.y = new_q.y();
  odom_msg.pose.pose.orientation.z = new_q.z();

  // [P2 FIX 2026-07-10] REP-105 / nav_msgs convention: the twist is expressed
  // in child_frame_id (= base_frame, the BODY frame), not the header frame.
  // The old world-frame twist made linear velocity wrong for any nonzero yaw
  // and the yaw-rate component wrong whenever the vehicle was pitched or
  // banked. Linear: rotate the world velocity into the body; angular: the
  // bias-corrected gyro is already the body rate.
  const Eigen::Vector3f v_lin_body = new_q.conjugate() * new_v_lin_w;
  odom_msg.twist.twist.linear.x = v_lin_body.x();
  odom_msg.twist.twist.linear.y = v_lin_body.y();
  odom_msg.twist.twist.linear.z = v_lin_body.z();
  odom_msg.twist.twist.angular.x = new_v_ang_b.x();
  odom_msg.twist.twist.angular.y = new_v_ang_b.y();
  odom_msg.twist.twist.angular.z = new_v_ang_b.z();

  // Pose covariance: diagonal only.
  // When GICP is accepted use sqrt(fitness) as a positional sigma (metres).
  // When dead-reckoning (consecutive GICP failures) inflate by elapsed time and
  // distance travelled (speed*time), not by missed-scan count (P3).
  {
    const double kBaseSigmaXY = 0.05;  // m   — floor for accepted scans
    const double kBaseSigmaZ = 0.10;   // m   — z less constrained by LiDAR
    const double kBaseSigmaRot = 0.01; // rad — roll/pitch/yaw floor
    const double kFitnessScale =
        1.0; // sigma_xy = max(base, scale * sqrt(fitness))
    // [P2 FIX 2026-07-09] NON-BLOCKING covariance snapshot. performLocalization
    // holds pose_mutex across the entire GICP solve (p50 18 ms, p99 68 ms);
    // taking it here — on EVERY IMU callback, in a MutuallyExclusive group —
    // stalled the whole IMU pipeline for up to gicp_ms each scan (output gaps
    // + delayed buffering => staler deskew clamp). try_lock instead: on
    // contention we publish the previous snapshot (covariance inputs change
    // once per scan; one-cycle staleness is immaterial). Statics are safe:
    // the IMU group is MutuallyExclusive.
    static bool cov_last_gicp_valid = false;
    static double cov_last_accepted_fitness = -1.0;
    static int cov_consecutive_failures = 0;
    static double cov_last_accepted_scan_stamp = -1.0;
    {
      std::unique_lock<std::mutex> pose_lock(this->pose_mutex,
                                             std::try_to_lock);
      if (pose_lock.owns_lock()) {
        cov_last_gicp_valid = this->last_gicp_valid_;
        cov_last_accepted_fitness = this->last_accepted_fitness_score_;
        cov_consecutive_failures = this->consecutive_failures_;
        cov_last_accepted_scan_stamp = this->last_accepted_scan_stamp_;
      }
    }

    double s_xy, s_z, s_rot;
    if (cov_last_gicp_valid && cov_last_accepted_fitness >= 0.0) {
      double f_sigma = kFitnessScale * std::sqrt(cov_last_accepted_fitness);
      s_xy = std::max(kBaseSigmaXY, f_sigma);
      s_z = std::max(kBaseSigmaZ, 2.0 * f_sigma);
      s_rot = std::max(kBaseSigmaRot, 0.1 * f_sigma);
    } else {
      s_xy = kBaseSigmaXY;
      s_z = kBaseSigmaZ;
      s_rot = kBaseSigmaRot;
    }

    // Dead-reckoning inflation (P3). While GICP is failing the estimate rides
    // on IMU integration, whose error grows with DISTANCE travelled
    // (speed*time) plus a slow time term -- not with the raw number of missed
    // scans. At racing speed N missed scans cover ~4x more ground at 200 mph
    // than at 100 mph, so a count-based term under-reports uncertainty exactly
    // when it matters. Keyed on consecutive_failures_ (the real dead-reckon
    // signal) and added on top of the base/fitness sigma above.
    // dr_cov_time_rate_ = dr_cov_dist_frac_ = 0 disables.
    if (cov_consecutive_failures > 0) {
      const double elapsed_dr =
          (cov_last_accepted_scan_stamp > 0.0)
              ? std::max(0.0, imu_local.stamp - cov_last_accepted_scan_stamp)
              : 0.0;
      const double speed = static_cast<double>(new_v_lin_w.norm());
      const double drift = this->dr_cov_time_rate_ * elapsed_dr +
                           this->dr_cov_dist_frac_ * speed * elapsed_dr;
      s_xy += drift;
      s_z += 2.0 * drift;
      s_rot += 0.05 * drift;
    }
    auto &c = odom_msg.pose.covariance;
    c.fill(0.0);
    c[0] = s_xy * s_xy;    // x
    c[7] = s_xy * s_xy;    // y
    c[14] = s_z * s_z;     // z
    c[21] = s_rot * s_rot; // roll
    c[28] = s_rot * s_rot; // pitch
    c[35] = s_rot * s_rot; // yaw
  }

  this->publishGicpOdom(odom_msg);
  this->publishNavSatFix(odom_msg);

  // Output-rate reporting is useful while tuning, but publishing itself is
  // IMU-rate and must not create a steady stream of terminal INFO messages.
  static int odom_publish_count = 0;
  static auto last_report_time = std::chrono::steady_clock::now();
  if (this->verbose_) {
    ++odom_publish_count;
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - last_report_time)
                             .count();
    if (elapsed >= 1000) { // Report every second in opt-in verbose mode.
      RCLCPP_INFO(this->get_logger(), "Odometry publish rate: %d Hz",
                  odom_publish_count);
      odom_publish_count = 0;
      last_report_time = now;
    }
  }

  if (this->imu_only_mode_) {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = new_q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = new_p;
  }

  // Don't publish TF from propagated state - only from GICP-corrected pose in
  // publishPose() High-frequency TF from IMU propagation drifts between GICP
  // corrections TF publishing is handled in publishPose() at GICP rate (15 Hz)
  // with corrected pose
}

void gicp_localizer::GicpLocalizer::updateState() {

  // Lock thread to prevent state from being accessed by propagateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);

  // Every accepted measurement must earn a fresh one-shot output. A stale
  // snapshot from the previous accepted scan must never be reused.
  this->latency_compensated_output_.valid = false;
  this->latency_compensated_output_.stationary = false;
  this->latency_compensated_output_.observer_epoch = this->observer_epoch_;
  this->latency_compensated_output_.status = "not_attempted";

  Eigen::Vector3f pin = this->basePose.p;
  Eigen::Quaternionf qin = this->basePose.q;
  const Eigen::Vector3f measurement_p = pin;
  const Eigen::Quaternionf measurement_q = qin.normalized();
  double dt = this->observer_dt_;

  // On very first update after initialization, dt might be large
  // Just skip the update but don't warn
  if (dt <= 0.0) {
    this->latency_compensated_output_.status = "invalid_observer_dt";
    return; // Skip invalid dt
  }

  if (dt > 1.0) {
    this->latency_compensated_output_.status = "scan_gap_too_large";
    RCLCPP_WARN(this->get_logger(),
                "Large dt in updateState: %.3f sec, skipping update", dt);
    return; // Skip if dt is too large (probably first update or dropped scans)
  }

  // Bound the effective timestep used for the proportional corrections. The
  // observer applies dt*K, which is forward-Euler and stable only for dt*K < 2;
  // at Kv=11.25 that bound is hit at dt≈0.18 s, so a 0.3-0.5 s scan gap
  // (dropped Luminar frames / high-speed racing) would otherwise inject an
  // unstable correction from a single GICP residual. At nominal ~10 Hz dt_eff
  // == dt.
  const double dt_eff = std::min(dt, this->geo_observer_dt_max_);

  // Validate inputs
  bool inputs_valid =
      std::isfinite(pin.x()) && std::isfinite(pin.y()) &&
      std::isfinite(pin.z()) && std::isfinite(qin.w()) &&
      std::isfinite(qin.x()) && std::isfinite(qin.y()) &&
      std::isfinite(qin.z()) && std::isfinite(this->state.p.x()) &&
      std::isfinite(this->state.p.y()) && std::isfinite(this->state.p.z()) &&
      std::isfinite(this->state.q.w()) && std::isfinite(this->state.q.x()) &&
      std::isfinite(this->state.q.y()) && std::isfinite(this->state.q.z());

  if (!inputs_valid) {
    this->latency_compensated_output_.status = "invalid_observer_state";
    RCLCPP_WARN(this->get_logger(),
                "Invalid inputs in updateState - pin=[%.3f,%.3f,%.3f] "
                "state.p=[%.3f,%.3f,%.3f]",
                pin.x(), pin.y(), pin.z(), this->state.p.x(), this->state.p.y(),
                this->state.p.z());
    return;
  }

  // Delayed-state correction target. basePose (pin/qin) is the GICP result at
  // the scan's MEDIAN POINT TIME; by now the live state has advanced another
  // 0.1-0.3 s (half sweep + queueing + solve). Pulling it toward that stale
  // absolute pose creates speed-dependent lag. Form instead
  //   T_corr = T_meas(scan_t) * inv(T_observer(scan_t))
  // and target T_corr * T_observer(now). The historical pose MUST come from
  // this same live observer. The independent scan-chain T_prior has different
  // velocity feedback and cannot observe accumulated high-rate odom drift.
  if (this->geo_delta_correction_) {
    ObserverPoseSample historical;
    Eigen::Vector3f delayed_target_p;
    Eigen::Quaternionf delayed_target_q;
    double corr_trans = std::numeric_limits<double>::infinity();
    double corr_angle_deg = std::numeric_limits<double>::infinity();
    const bool have_historical_pose = interpolateObserverPose(
        this->observer_pose_history_, this->t_prior_stamp_, 0.1, &historical);
    const ObserverPoseSample* newest =
        this->observer_pose_history_.empty()
            ? nullptr
            : &this->observer_pose_history_.back();
    const bool epoch_compatible =
        have_historical_pose && newest &&
        observerEpochCompatible(historical, *newest, this->observer_epoch_,
                                this->registration_observer_epoch_);
    const bool have_delayed_target =
        epoch_compatible &&
        delayedObserverCorrectionTarget(
            historical, pin, qin, this->state.p, this->state.q,
            &delayed_target_p, &delayed_target_q, &corr_trans, &corr_angle_deg);
    double output_delay_s = 0.0;
    const double newest_observer_stamp = newest ? newest->stamp : 0.0;
    const bool output_delay_bounded = observerDelayWithinBound(
        this->t_prior_stamp_, newest_observer_stamp,
        this->output_max_imu_delay_compensation_s_, &output_delay_s);

    std::string rejection_reason;
    if (!have_historical_pose) {
      rejection_reason = "observer_history_unavailable";
    } else if (!epoch_compatible) {
      rejection_reason = "observer_epoch_mismatch";
    } else if (!have_delayed_target) {
      rejection_reason = "delayed_target_invalid";
    } else if (!std::isfinite(corr_trans) || corr_trans >= 100.0 ||
               !std::isfinite(corr_angle_deg) || corr_angle_deg >= 45.0) {
      rejection_reason = "observer_correction_implausible";
    } else if (!output_delay_bounded) {
      rejection_reason = "delay_out_of_bounds";
    }

    WheelMotionIntegral wheel_motion;
    if (rejection_reason.empty() &&
        (this->output_wheel_distance_anchor_enabled_ ||
         this->output_motion_gate_enabled_)) {
      if (!this->wheel_speed_topic_.empty()) {
        std::lock_guard<std::mutex> wheel_lock(this->wheel_speed_mtx_);
        wheel_motion = integrateWheelSpeedDistance(
            this->wheel_speed_buffer_, this->t_prior_stamp_,
            newest_observer_stamp,
            this->output_motion_gate_wheel_endpoint_max_age_s_);
      }
    }

    Eigen::Vector3f product_target_p = delayed_target_p;
    if (rejection_reason.empty() &&
        this->output_wheel_distance_anchor_enabled_ &&
        !wheelAnchoredPlanarCompensationTarget(
            measurement_p, measurement_q, delayed_target_p, delayed_target_q,
            wheel_motion, &product_target_p)) {
      rejection_reason = "wheel_distance_anchor_invalid";
    }

    CompensationMotionGateResult motion_gate;
    if (rejection_reason.empty() && this->output_motion_gate_enabled_) {
      motion_gate = evaluateCompensationMotion(
          measurement_p, measurement_q, product_target_p, delayed_target_q,
          output_delay_s, wheel_motion,
          this->output_motion_gate_distance_abs_tolerance_m_,
          this->output_motion_gate_distance_rel_tolerance_,
          this->output_motion_gate_reverse_tolerance_m_,
          this->output_motion_gate_lateral_abs_tolerance_m_,
          this->output_motion_gate_lateral_rel_tolerance_,
          this->output_motion_gate_yaw_abs_tolerance_rad_,
          this->output_motion_gate_max_yaw_rate_rad_s_);
      if (!motion_gate.accepted()) {
        rejection_reason = compensationMotionGateReasonName(motion_gate.reason);
      }
    }

    if (rejection_reason.empty()) {
      pin = delayed_target_p;
      qin = delayed_target_q;

      // Product output is the exact accepted GICP pose advanced only by the
      // recent observer-relative motion. Do not use the proportional observer
      // state here: Kp/Kq smoothing would leave a speed-dependent residual.
      const Eigen::Vector3f v_lin_body =
          this->state.q.conjugate() * this->state.v.lin.w;
      this->latency_compensated_output_.valid = true;
      this->latency_compensated_output_.stationary = false;
      this->latency_compensated_output_.observer_epoch =
          this->observer_epoch_;
      this->latency_compensated_output_.status =
          this->output_wheel_distance_anchor_enabled_
              ? "wheel_distance_anchored"
              : "compensated";
      this->latency_compensated_output_.measurement_stamp =
          this->t_prior_stamp_;
      this->latency_compensated_output_.output_stamp = newest_observer_stamp;
      this->latency_compensated_output_.measurement_p = measurement_p;
      this->latency_compensated_output_.measurement_q = measurement_q;
      this->latency_compensated_output_.p = product_target_p;
      this->latency_compensated_output_.q = delayed_target_q.normalized();
      this->latency_compensated_output_.v_lin_body = v_lin_body;
      this->latency_compensated_output_.v_ang_body = this->state.v.ang.b;
    } else {
      this->latency_compensated_output_.status = rejection_reason;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Latency compensation rejected (%s): epoch=[history=%lu newest=%lu "
          "current=%lu solve=%lu] delay=%.3fs delta=[%.2f,%.2f]m "
          "wheel=%.2fm yaw=%.1fdeg; publishing raw GICP and isolating "
          "observer history",
          rejection_reason.c_str(),
          static_cast<unsigned long>(historical.epoch),
          static_cast<unsigned long>(newest ? newest->epoch : 0),
          static_cast<unsigned long>(this->observer_epoch_),
          static_cast<unsigned long>(this->registration_observer_epoch_),
          output_delay_s, static_cast<double>(motion_gate.delta_body.x()),
          static_cast<double>(motion_gate.delta_body.y()),
          motion_gate.expected_distance_m,
          motion_gate.yaw_delta_rad * 180.0 / M_PI);

      // Do not pull the live observer toward a stale absolute measurement when
      // its delayed relative motion cannot be trusted. Start a fresh epoch at
      // the current state so the next accepted scan can build a clean bridge.
      ++this->observer_epoch_;
      this->observer_pose_history_.clear();
      if (newest_observer_stamp > 0.0) {
        this->observer_pose_history_.push_back(
            {newest_observer_stamp, this->state.p, this->state.q,
             this->observer_epoch_});
      }
      ++this->geo.update_seq;
      return;
    }
  }

  const bool planar_z_locked =
      this->gicp_dof_mode_ == "planar" && this->track_elevation_enabled_ &&
      this->track_z_hold_valid_.load(std::memory_order_acquire);
  if (planar_z_locked) {
    // Internal Z follows only the accepted static-TTL registration height.
    // Neither the observer correction nor IMU double integration may estimate
    // it; the separately frozen product-output Z is applied when publishing.
    const float locked_z = static_cast<float>(
        this->track_z_hold_m_.load(std::memory_order_relaxed));
    pin.z() = locked_z;
    this->state.p.z() = locked_z;
    this->state.v.lin.w.z() = 0.0f;
  }

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Construct error quaternion
  qe = qhat.conjugate() * qin;

  // P1 yaw-safety fix #3: bound the per-update ORIENTATION correction the way
  // position/velocity already can be bounded. qe is the body-frame rotation
  // error the observer will pull toward with gain dt_eff*Kq (~0.45/update);
  // without a clamp, one bad accepted scan with a large heading error injects
  // dt_eff*Kq*err yaw in a single update (runs 19/20: tens of degrees). Clamp
  // the error's yaw component (body z ~ heading on a near-level vehicle) and
  // optionally its total magnitude BEFORE the gain is applied. Legitimate
  // corrections converge unaffected: a clamped 5 deg yaw error still pulls
  // ~2.3 deg/update at 10 Hz — over 20 deg/s of authority.
  if (this->geo_max_yaw_correction_deg_ > 0.0 ||
      this->geo_max_rot_correction_deg_ > 0.0) {
    constexpr float kDeg2RadF = static_cast<float>(M_PI / 180.0);
    Eigen::AngleAxisf aa_e(qe);
    Eigen::Vector3f rv = aa_e.angle() * aa_e.axis();
    bool clamped = false;
    if (this->geo_max_yaw_correction_deg_ > 0.0) {
      const float max_yaw =
          static_cast<float>(this->geo_max_yaw_correction_deg_) * kDeg2RadF;
      if (std::abs(rv.z()) > max_yaw) {
        rv.z() = std::copysign(max_yaw, rv.z());
        clamped = true;
      }
    }
    if (this->geo_max_rot_correction_deg_ > 0.0) {
      const float max_rot =
          static_cast<float>(this->geo_max_rot_correction_deg_) * kDeg2RadF;
      const float n = rv.norm();
      if (n > max_rot) {
        rv *= max_rot / n;
        clamped = true;
      }
    }
    if (clamped) {
      const float ang = rv.norm();
      qe = (ang > 1e-9f) ? Eigen::Quaternionf(Eigen::AngleAxisf(ang, rv / ang))
                         : Eigen::Quaternionf::Identity();
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "updateState: orientation error clamped (yaw<=%.1fdeg, rot<=%.1fdeg) "
          "before applying observer gain — inspect yaw_innovation_deg",
          this->geo_max_yaw_correction_deg_, this->geo_max_rot_correction_deg_);
    }
  }

  double sgn = 1.0;
  if (qe.w() < 0) {
    sgn = -1.0;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - fabs(qe.w());
  qcorr.vec() = sgn * qe.vec();
  qcorr = qhat * qcorr;

  // Position error
  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  // Optional online bias adaptation. Keep disabled by default for fused
  // Point One (Atlas) INS input so GICP residuals do not chase drift by
  // rewriting the trusted IMU bias estimate. Setting Kab/Kgb > 0 restores the
  // upstream DLIO adaptive observer behavior.
  if (this->geo_Kab_ > 0.0) {
    const double abias_max = this->geo_abias_max_;
    this->state.b.accel -= dt_eff * this->geo_Kab_ * err_body;
    this->state.b.accel =
        this->state.b.accel.array().min(abias_max).max(-abias_max);
  }

  if (this->geo_Kgb_ > 0.0) {
    const double gbias_max = this->geo_gbias_max_;
    this->state.b.gyro[0] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.x();
    this->state.b.gyro[1] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.y();
    this->state.b.gyro[2] -= dt_eff * this->geo_Kgb_ * qe.w() * qe.z();
    this->state.b.gyro =
        this->state.b.gyro.array().min(gbias_max).max(-gbias_max);
  }

  // Proportional observer correction (matching upstream DLIO design), using the
  // bounded dt_eff and optional hard clamps on the per-update correction
  // magnitude so one GICP residual after a gap can't yank the state.
  // Position correction
  Eigen::Vector3f pos_corr = dt_eff * this->geo_Kp_ * err;
  if (this->geo_max_pos_correction_ > 0.0) {
    const float n = pos_corr.norm();
    if (n > this->geo_max_pos_correction_) {
      pos_corr *= static_cast<float>(this->geo_max_pos_correction_ / n);
    }
  }
  this->state.p += pos_corr;

  // Velocity correction
  // This corrects the asynchronous IMU-rate observer. The cleanup-compatible
  // residual/dt feedback belongs to the separate next-scan integration seed
  // (prev_vel) and is applied by the accepted registration path; it must not
  // be written into this live observer state.
  Eigen::Vector3f vel_corr = dt_eff * this->geo_Kv_ * err;
  if (this->geo_max_vel_correction_ > 0.0) {
    const float n = vel_corr.norm();
    if (n > this->geo_max_vel_correction_) {
      vel_corr *= static_cast<float>(this->geo_max_vel_correction_ / n);
    }
  }
  this->state.v.lin.w += vel_corr;

  // Ground vehicle constraint: damp vertical velocity toward zero.
  // A ground vehicle's true Z-velocity is ~0; residual gravity miscompensation
  // causes vel_z to drift. Apply exponential decay each update.
  this->state.v.lin.w[2] *= (1.0f - dt_eff * this->geo_Kz_damping_);
  if (planar_z_locked) {
    this->state.p.z() = pin.z();
    this->state.v.lin.w.z() = 0.0f;
  }
  if (this->geo_max_state_speed_ > 0.0) {
    const float speed = this->state.v.lin.w.norm();
    if (std::isfinite(speed) &&
        speed > static_cast<float>(this->geo_max_state_speed_)) {
      this->state.v.lin.w *=
          static_cast<float>(this->geo_max_state_speed_) / speed;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "updateState: observer speed %.1fm/s exceeded physical cap %.1fm/s; "
          "clamped",
          static_cast<double>(speed), this->geo_max_state_speed_);
    }
  }

  // Orientation correction
  this->state.q.w() += dt_eff * this->geo_Kq_ * qcorr.w();
  this->state.q.vec() += dt_eff * this->geo_Kq_ * qcorr.vec();
  this->state.q.normalize();

  // Validate updated state
  bool state_valid_after =
      std::isfinite(this->state.p.x()) && std::isfinite(this->state.p.y()) &&
      std::isfinite(this->state.p.z()) && std::isfinite(this->state.q.w()) &&
      std::isfinite(this->state.v.lin.w.x()) &&
      std::isfinite(this->state.v.lin.w.y()) &&
      std::isfinite(this->state.v.lin.w.z());

  if (!state_valid_after) {
    RCLCPP_ERROR(
        this->get_logger(),
        "State became invalid after update! Resetting to GICP measurement.");
    // Reset to valid GICP measurement
    this->state.p = pin;
    this->state.q = qin;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    // [REVIEW FIX 2026-07-08 P3] Keep the calibrated biases through the
    // emergency reset when they are still finite — they are calibration, not
    // drifting state. Zero them only if uncalibrated or themselves corrupt.
    if (!this->imu_calibrated_.load() || !this->state.b.accel.allFinite() ||
        !this->state.b.gyro.allFinite()) {
      this->state.b.accel = Eigen::Vector3f::Zero();
      this->state.b.gyro = Eigen::Vector3f::Zero();
    }
  }

  // Store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;
  ++this->geo.update_seq; // Signal propagateState to discard stale computations

  // The correction changed the live state at the newest observer timestamp.
  // Older history belongs to the pre-correction trajectory and must not be
  // reused by the next delayed scan. Preserve one corrected anchor at the
  // same newest timestamp; subsequent IMU samples extend it normally.
  if (!this->observer_pose_history_.empty()) {
    const double newest_stamp = this->observer_pose_history_.back().stamp;
    ++this->observer_epoch_;
    this->observer_pose_history_.clear();
    this->observer_pose_history_.push_back(
        {newest_stamp, this->state.p, this->state.q, this->observer_epoch_});
  }

  // This contains live position, velocity, and bias estimates.  It is an
  // opt-in development trace, not a normal runtime diagnostic.
  static int update_count = 0;
  if (this->verbose_ && ++update_count % 20 == 0) {
    RCLCPP_INFO(
        this->get_logger(),
        "Geo Observer | pos_err=[%.3f,%.3f,%.3f]m vel=[%.2f,%.2f,%.2f]m/s | "
        "bias_gyro=[%.4f,%.4f,%.4f] bias_accel=[%.3f,%.3f,%.3f]",
        err.x(), err.y(), err.z(), this->state.v.lin.w.x(),
        this->state.v.lin.w.y(), this->state.v.lin.w.z(),
        this->state.b.gyro.x(), this->state.b.gyro.y(), this->state.b.gyro.z(),
        this->state.b.accel.x(), this->state.b.accel.y(),
        this->state.b.accel.z());
  }

  RCLCPP_DEBUG(this->get_logger(),
               "Geo Observer: pos_err=[%.3f,%.3f,%.3f] vel=[%.2f,%.2f,%.2f] "
               "bias_a=[%.3f,%.3f,%.3f]",
               err.x(), err.y(), err.z(), this->state.v.lin.w.x(),
               this->state.v.lin.w.y(), this->state.v.lin.w.z(),
               this->state.b.accel.x(), this->state.b.accel.y(),
               this->state.b.accel.z());
}
