#include "gicp_interface/detail/localizer_utils.hpp"

using gicp_localizer::detail::matrixFinite;
using gicp_localizer::detail::rotationDistanceDeg;

void gicp_localizer::GicpLocalizer::callbackGtOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  const double stamp =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;
  const auto& linear = msg->twist.twist.linear;
  const auto& angular = msg->twist.twist.angular;
  const double q_norm_sq =
      q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  const bool pose_covariance_finite = std::all_of(
      msg->pose.covariance.begin(), msg->pose.covariance.end(),
      [](double value) { return std::isfinite(value); });
  const bool twist_covariance_finite = std::all_of(
      msg->twist.covariance.begin(), msg->twist.covariance.end(),
      [](double value) { return std::isfinite(value); });
  const bool fields_finite =
      std::isfinite(stamp) && stamp > 0.0 &&
      std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::isfinite(q.w) && std::isfinite(q.x) &&
      std::isfinite(q.y) && std::isfinite(q.z) &&
      std::isfinite(q_norm_sq) && q_norm_sq > 1e-12 &&
      std::isfinite(linear.x) && std::isfinite(linear.y) &&
      std::isfinite(linear.z) && std::isfinite(angular.x) &&
      std::isfinite(angular.y) && std::isfinite(angular.z) &&
      pose_covariance_finite && twist_covariance_finite;
  if (!fields_finite) {
    const auto count = ++this->gt_dropped_invalid_;
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping invalid GT odom before quaternion normalization "
        "(stamp=%.9f q_norm_sq=%.6g, gt_dropped_invalid=%lu)",
        stamp, q_norm_sq, static_cast<unsigned long>(count));
    return;
  }

  const bool frame_ok =
      !msg->header.frame_id.empty() && !msg->child_frame_id.empty() &&
      (this->gt_expected_frame_id_.empty() ||
       msg->header.frame_id == this->gt_expected_frame_id_) &&
      (this->gt_expected_child_frame_id_.empty() ||
       msg->child_frame_id == this->gt_expected_child_frame_id_);
  if (!frame_ok) {
    const auto count = ++this->gt_dropped_frame_;
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping GT odom with unexpected frames '%s' -> '%s'; expected "
        "'%s' -> '%s' (gt_dropped_frame=%lu)",
        msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
        this->gt_expected_frame_id_.c_str(),
        this->gt_expected_child_frame_id_.c_str(),
        static_cast<unsigned long>(count));
    return;
  }

  GtSample s;
  s.stamp = stamp;
  s.p = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z) + this->ins_offset_;
  s.q = Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  s.q.normalize();
  s.v_lin_body = Eigen::Vector3f(msg->twist.twist.linear.x,
                                 msg->twist.twist.linear.y,
                                 msg->twist.twist.linear.z);
  s.v_ang_body = Eigen::Vector3f(msg->twist.twist.angular.x,
                                 msg->twist.twist.angular.y,
                                 msg->twist.twist.angular.z);
  // Carry Atlas-reported position covariance per-sample. The RTK quality
  // gate is no longer applied here -- every sample is pushed into the buffer
  // regardless of FIXED/FLOAT/dead-reckoning state. The gate now runs at
  // the CONSUMER side:
  //   * tryRtkCalibrationStep (init/calibration)  -> require FIXED
  //   * maybeSnapPoseToGT (recovery from GICP failure) -> accept ANY sample
  // Rationale: Atlas's onboard INS already does coupled GNSS+IMU dead-
  // reckoning with calibrated sensors during RTK loss. When GICP fails to
  // match the LiDAR scan, the next-best truth is Atlas's pose at whatever
  // quality it currently has -- not our own software IMU dead-reckoning.
  s.cov_pos_xx = msg->pose.covariance[0];
  s.cov_pos_yy = msg->pose.covariance[7];
  s.cov_pos_zz = msg->pose.covariance[14];
  // [REVIEW FIX 2026-07-08] Carry the yaw variance (rad^2) the adapter
  // publishes in pose.covariance[35] (Atlas rpy covariance, deg^2 -> rad^2).
  // Consumed by the INS heading prior's yaw-quality gate. Generic publishers
  // that leave the field at 0 are treated as "unpopulated" downstream.
  // [P2 FIX 2026-07-10] NaN yaw covariance is KNOWN-BAD (invalid heading
  // solution), not "unpopulated": the gate's `cov_yaw > 0` test is false for
  // NaN, so it used to fail OPEN. Map NaN to +inf so the yaw-sigma gate
  // rejects it (mirrors GLIM gnss_global's sanitize_yaw_var; the adapter now
  // also sanitizes at the source, this is defense in depth for other GT
  // publishers). 0/negative keep the documented "unpopulated passes" compat.
  s.cov_yaw = std::isnan(msg->pose.covariance[35])
      ? std::numeric_limits<double>::infinity()
      : msg->pose.covariance[35];

  // Cache base_frame ← gt_body_frame TF on the first message (mirrors the IMU
  // extrinsic caching pattern in callbackImu). Required before the snap helper
  // and other GT consumers can compose poses; callback keeps appending
  // samples even while TF is missing.  Runs BEFORE the odom-init block below
  // so composeGtPoseInBase has the extrinsic ready to bring the first GT
  // sample into base_frame coordinates before applyInitialPose seeds the state.
  if (!this->gt_extrinsics_cached_.load()) {
    // [P2 FIX 2026-07-09] Serialize: this callback runs in a REENTRANT group,
    // so two first messages could execute this block concurrently
    // (std::string assignment race = UB, torn T_base_gtbody_ publication).
    // Re-check under the lock; gt_extrinsics_cached_ (atomic) is written LAST.
    std::lock_guard<std::mutex> init_lock(this->gt_init_mtx_);
    if (!this->gt_extrinsics_cached_.load()) {
    if (this->gt_body_frame_.empty()) {
      this->gt_body_frame_ = msg->child_frame_id;
      if (this->gt_body_frame_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GT odom message has empty child_frame_id; assuming GT body == base_frame ('%s')",
                             this->base_frame.c_str());
        this->gt_body_frame_ = this->base_frame;
      }
    }
    if (this->gt_body_frame_ == this->base_frame) {
      this->T_base_gtbody_.setIdentity();
      this->gt_extrinsics_cached_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "GT recovery: gt_body == base_frame ('%s'), using identity extrinsic",
                  this->base_frame.c_str());
    } else {
      try {
        auto tf_bg = this->tf_buffer->lookupTransform(
            this->base_frame, this->gt_body_frame_, tf2::TimePointZero);
        Eigen::Quaternionf q_bg(
            tf_bg.transform.rotation.w, tf_bg.transform.rotation.x,
            tf_bg.transform.rotation.y, tf_bg.transform.rotation.z);
        Eigen::Vector3f t_bg(
            tf_bg.transform.translation.x, tf_bg.transform.translation.y,
            tf_bg.transform.translation.z);
        this->T_base_gtbody_.setIdentity();
        this->T_base_gtbody_.block<3, 3>(0, 0) = q_bg.toRotationMatrix();
        this->T_base_gtbody_.block<3, 1>(0, 3) = t_bg;
        this->gt_extrinsics_cached_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "GT recovery: cached %s ← %s extrinsic: t=[%.3f,%.3f,%.3f]",
                    this->base_frame.c_str(), this->gt_body_frame_.c_str(),
                    t_bg.x(), t_bg.y(), t_bg.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GT recovery: cannot cache %s ← %s TF: %s — deferring snap",
                             this->base_frame.c_str(), this->gt_body_frame_.c_str(), ex.what());
      }
    }
    }  // re-check scope (gt_init_mtx_ held)
  }

  // Odom init: on the first GT odom message (after the TF cache above is
  // populated), seed state from GT so the node starts at the correct location
  // even when the bag begins mid-run.  Composes through T_base_gtbody_ so the
  // seeded state.p lands at base_frame, not at gt_body_frame -- otherwise an
  // off-base gt_odom source would seed the state with a constant lever-arm
  // offset. For the
  // AV-24 single-source P1 config the composition is identity since gt_body
  // == base_frame == gps_antenna_top. If the extrinsic hasn't cached yet (TF lookup
  // deferred), skip this message and try again on the next one rather than
  // seeding from a frame we can't compose.
  // Overrides any param-based initial pose. Sets first_opt_done so odom starts
  // publishing immediately without waiting for the first accepted GICP scan.
  if (this->use_odom_init_ && !this->use_odom_init_applied_.load()) {
    // [P2 FIX 2026-07-09] Serialize + re-check: without this, two concurrent
    // GT callbacks could both pass the flag test and run applyInitialPose
    // twice, interleaving their (sequential) lock scopes.
    std::lock_guard<std::mutex> init_lock(this->gt_init_mtx_);
    Eigen::Vector3f init_p;
    Eigen::Quaternionf init_q;
    // Re-check under the lock: a concurrent callback may have seeded while
    // we waited. The && short-circuit skips the compose entirely then.
    if (!this->use_odom_init_applied_.load() && this->composeGtPoseInBase(s, init_p, init_q)) {
      this->use_odom_init_applied_ = true;
      const rclcpp::Time stamp_ros(msg->header.stamp.sec, msg->header.stamp.nanosec);
      // [P3 FIX 2026-07-14] Seed velocity from the GT message's own twist so a
      // mid-run start does not dead-reckon from v=0 (which makes the first IMU
      // prior integrate from zero and immediately re-fail). Compose the twist
      // into base frame and rotate the linear part into world with init_q.
      Eigen::Vector3f init_v_lin_base_body, init_v_ang_base_body;
      const bool have_twist =
          this->composeGtTwistInBase(s, init_v_lin_base_body, init_v_ang_base_body);
      const Eigen::Vector3f init_v_world = init_q * init_v_lin_base_body;
      this->applyInitialPose(init_p, init_q, stamp_ros, "gt_odom",
                             have_twist ? &init_v_world : nullptr);
      {
        std::lock_guard<std::mutex> lock(this->geo.mtx);
        this->geo.first_opt_done = true;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Odom init: pose set from GT odom at t=%.3f gt_pos=[%.2f,%.2f,%.2f] "
                  "-> base_pos=[%.2f,%.2f,%.2f] (gt_body='%s')",
                  s.stamp, s.p.x(), s.p.y(), s.p.z(),
                  init_p.x(), init_p.y(), init_p.z(),
                  this->gt_body_frame_.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Odom init: deferring -- gt_body -> base extrinsic not cached yet");
    }
  }

  std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
  if (!this->gt_odom_buffer_.empty() && s.stamp <= this->gt_odom_buffer_.back().stamp) {
    // [P1 FIX 2026-07-14] A large backward jump is an epoch reset (bag loop /
    // adapter re-anchor / power-cycle): flag a COORDINATED estimator reset
    // (performed on the IMU thread — see resetEstimatorForEpochChange). Do NOT
    // clear gt_odom_buffer_ alone here: with the observer seeds / calibration
    // timers still on the old epoch, GT lookups would read a fresh buffer with
    // old-epoch scan stamps and the INS prior / snap would
    // silently misbehave. Small regressions are dropped as out-of-order.
    if (this->gt_odom_buffer_.back().stamp - s.stamp > 5.0 && !this->epoch_reset_pending_.load()) {
      this->epoch_reset_regress_s_.store(this->gt_odom_buffer_.back().stamp - s.stamp);
      this->epoch_reset_pending_.store(true);
      RCLCPP_WARN(this->get_logger(),
                  "GT odom stamp EPOCH RESET detected (rewind %.3f s → %.3f); scheduling coordinated "
                  "estimator re-initialization.",
                  this->gt_odom_buffer_.back().stamp - s.stamp, s.stamp);
    }
    // Out-of-order or duplicate timestamp; drop to keep buffer monotone.
    return;
  }
  this->gt_odom_buffer_.push_back(s);
  while (this->gt_odom_buffer_.size() > this->gt_odom_buffer_size_) {
    this->gt_odom_buffer_.pop_front();
  }
  if (!this->gt_odom_received_.exchange(true)) {
    RCLCPP_INFO(this->get_logger(),
                "First ground-truth odom received at stamp=%.3f frame=%s child_frame=%s",
                s.stamp, msg->header.frame_id.c_str(),
                msg->child_frame_id.empty() ? "(empty)" : msg->child_frame_id.c_str());
  }
}

bool gicp_localizer::GicpLocalizer::gtSampleIsRtkFixed(const GtSample& s) const {
  // [P3 FIX 2026-07-10] NAMING CAVEAT: despite the name, this is a
  // COVARIANCE-QUALITY gate, not a solution-type check — the adapter does not
  // forward solution_type per-sample. An RTK-FLOAT sample whose reported
  // position variance happens to sit under the thresholds (~3.2 cm sigma xy
  // at the shipped 1e-3) passes "as fixed" BY DESIGN: consumers gate on
  // accuracy, not on the fix flag. Snap recovery intentionally bypasses this
  // gate entirely.
  // When the gate is disabled, treat every sample as FIXED -- the operator
  // has explicitly opted into "trust whatever the upstream publishes".
  if (!this->rtk_gate_enabled_) return true;
  // [P1 FIX 2026-07-14] finite + nonnegative + thresholded (rtk_gate.hpp):
  // the former plain `<=` accepted a finite negative sentinel (-1 =
  // "covariance not populated") as RTK-quality, so unknown-quality samples
  // could drive the INS heading prior and bias calibration. Parity with the
  // adapter's filtered_odom_rtk_fixed gate.
  return rtkPositionCovarianceOk(s.cov_pos_xx, s.cov_pos_yy, s.cov_pos_zz,
                                 this->rtk_gate_max_pose_var_xy_,
                                 this->rtk_gate_max_pose_var_z_,
                                 this->rtk_gate_allow_zero_covariance_);
}

bool gicp_localizer::GicpLocalizer::getGtPoseAt(double stamp, GtSample& out) {
  std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
  if (this->gt_odom_buffer_.size() < 2) {
    if (this->gt_odom_buffer_.size() == 1 &&
        std::abs(this->gt_odom_buffer_.front().stamp - stamp) <= this->gt_odom_max_dt_) {
      out = this->gt_odom_buffer_.front();
      return true;
    }
    return false;
  }
  // Buffer is monotone non-decreasing. Find the first sample with stamp >= query.
  auto it = std::lower_bound(
      this->gt_odom_buffer_.begin(), this->gt_odom_buffer_.end(), stamp,
      [](const GtSample& s, double t) { return s.stamp < t; });

  if (it == this->gt_odom_buffer_.begin()) {
    if (std::abs(it->stamp - stamp) > this->gt_odom_max_dt_) return false;
    out = *it; return true;
  }
  if (it == this->gt_odom_buffer_.end()) {
    auto last = std::prev(it);
    if (std::abs(last->stamp - stamp) > this->gt_odom_max_dt_) return false;
    out = *last; return true;
  }
  auto a = std::prev(it);
  auto b = it;
  const double dt_total = b->stamp - a->stamp;
  // [P2 FIX 2026-07-14] Bound the BRACKET WIDTH, not just the nearer endpoint.
  // gt_odom_max_dt_ only limits the closer of the two bracketing samples, so a
  // query landing mid-gap across an unbounded GT dropout returned a chord pose
  // (up to metres / degrees through a curve) whose conservatively-combined
  // covariance can still pass the RTK gate and then feed the INS heading prior,
  // calibration. Reject interpolation across a large gap
  // (mirrors GLIM's max_interp_gap_sec P1 fix).
  if (dt_total <= 0.0 || dt_total > this->gt_interp_max_gap_ ||
      std::min(stamp - a->stamp, b->stamp - stamp) > this->gt_odom_max_dt_) {
    return false;
  }
  const float u = static_cast<float>((stamp - a->stamp) / dt_total);
  out.stamp = stamp;
  out.p = (1.0f - u) * a->p + u * b->p;
  out.q = a->q.slerp(u, b->q).normalized();
  out.v_lin_body = (1.0f - u) * a->v_lin_body + u * b->v_lin_body;
  out.v_ang_body = (1.0f - u) * a->v_ang_body + u * b->v_ang_body;
  // Covariance is not linearly interpolated: combine the two bracketing
  // samples conservatively so an interpolated pose can only pass the RTK
  // gate when BOTH neighbours were RTK-FIXED.
  // [P1 FIX 2026-07-14b] std::max() was NOT conservative here: max(valid,-1)
  // == valid, and max(valid, NaN) returns the valid first argument — a pose
  // interpolated against an unknown/invalid endpoint could pass
  // rtkPositionCovarianceOk() and feed the INS heading prior, RTK bias
  // calibration. rtkCombineInterpolatedVariance()
  // returns +inf (fails closed) when either endpoint is non-finite or
  // negative. (Endpoint/single-sample branches above copy a real sample
  // whole, so their covariance is gated as-is.)
  out.cov_pos_xx = rtkCombineInterpolatedVariance(a->cov_pos_xx, b->cov_pos_xx);
  out.cov_pos_yy = rtkCombineInterpolatedVariance(a->cov_pos_yy, b->cov_pos_yy);
  out.cov_pos_zz = rtkCombineInterpolatedVariance(a->cov_pos_zz, b->cov_pos_zz);
  // Yaw variance keeps the plain conservative-max COMPATIBILITY policy
  // (deliberately different from position): the yaw-quality gate treats
  // negative as "unpopulated, passes", so max(valid, -1) == valid is the
  // intended behavior for heading — position quality is what qualifies a
  // sample as RTK, yaw quality only gates the heading prior.
  out.cov_yaw = std::max(a->cov_yaw, b->cov_yaw);
  return true;
}

bool gicp_localizer::GicpLocalizer::getGtFiniteDiffVelWorld(
    double stamp, Eigen::Vector3f& v_world_out) {
  std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
  if (this->gt_odom_buffer_.size() < 2) return false;
  // Buffer is monotone non-decreasing (out-of-order samples dropped at insert).
  auto it = std::lower_bound(
      this->gt_odom_buffer_.begin(), this->gt_odom_buffer_.end(), stamp,
      [](const GtSample& s, double t) { return s.stamp < t; });
  // Choose a bracketing (or nearest adjacent) pair around the query stamp.
  auto b = (it == this->gt_odom_buffer_.end()) ? std::prev(it) : it;
  auto a = (b == this->gt_odom_buffer_.begin()) ? b : std::prev(b);
  if (a == b) b = std::next(b);  // query before first sample: use first pair
  const double dt = b->stamp - a->stamp;
  if (dt <= 1e-6) return false;
  // [P2 FIX 2026-07-14] Bound the bracket width too (see getGtPoseAt): a finite
  // difference taken across an unbounded GT dropout yields a chord velocity
  // that backfills the snap twist with a wrong value.
  if (dt > this->gt_interp_max_gap_) return false;
  // Both endpoints must be reasonably close to the query, mirroring
  // getGtPoseAt's staleness contract.
  if (std::min(std::abs(stamp - a->stamp), std::abs(b->stamp - stamp)) >
      this->gt_odom_max_dt_) {
    return false;
  }
  v_world_out = (b->p - a->p) / static_cast<float>(dt);
  return v_world_out.allFinite();
}

bool gicp_localizer::GicpLocalizer::composeGtPoseInBase(
    const GtSample& gt, Eigen::Vector3f& p_out,
    Eigen::Quaternionf& q_out) const {
  if (!this->gt_extrinsics_cached_) {
    // Extrinsic not cached yet (first message hasn't fully run the cache
    // block, or TF lookup deferred).  Caller decides whether to fall back
    // to gt.p/gt.q directly or skip this cycle.
    return false;
  }
  // T_map_base = T_map_gtbody * inv(T_base_gtbody).  Decomposed:
  //   q_out = gt.q * inv(R_base_gtbody)
  //   p_out = gt.p - q_out * t_base_gtbody
  // Same math as the snap helper -- factored out so recovery and the
  // first-message applyInitialPose path use identical composition rather than
  // re-deriving (or skipping) the lever-arm.
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Quaternionf q_gtbody_in_base(R_base_gtbody);
  q_out = (gt.q * q_gtbody_in_base.conjugate()).normalized();
  p_out = gt.p - q_out * t_base_gtbody;
  return true;
}

bool gicp_localizer::GicpLocalizer::composeGtTwistInBase(
    const GtSample& gt, Eigen::Vector3f& v_lin_body_out,
    Eigen::Vector3f& v_ang_body_out) const {
  if (!this->gt_extrinsics_cached_) {
    return false;
  }
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Matrix3f R_gtbody_base = R_base_gtbody.transpose();
  // Base origin position expressed in gt_body coords (translation of the inverse
  // transform); the lever-arm cross product below is therefore in gt_body coords.
  const Eigen::Vector3f t_gtbody_base = -R_gtbody_base * t_base_gtbody;

  // [P3 FIX 2026-07-14] Rotate the gt_body-frame twist INTO base coordinates
  // with R_base_gtbody (T_base_gtbody maps gt_body -> base, so a free vector
  // transforms v_base = R_base_gtbody * v_gtbody). The former code used the
  // TRANSPOSE R_gtbody_base, which applies the inverse rotation — wrong frame
  // whenever the gt_body -> base rotation is not identity (identity on AV-24
  // today, so latent, but a real bug for any off-base gt source).
  v_ang_body_out = R_base_gtbody * gt.v_ang_body;
  v_lin_body_out = R_base_gtbody * (gt.v_lin_body + gt.v_ang_body.cross(t_gtbody_base));
  return true;
}

// RTK-driven IMU bias calibration. Pairs each IMU sample with a time-matched GT
// pose/twist and accumulates the bias residual. Linear acceleration in world
// frame is estimated by finite-differencing v_world between successive paired
// samples. On window fill, biases are averaged, the state is seeded from the
// latest GT, imu_calibrated_ is flipped, and the function returns true.
bool gicp_localizer::GicpLocalizer::tryRtkCalibrationStep(
    double stamp, const Eigen::Vector3f& measured_gyro,
    const Eigen::Vector3f& measured_accel) {
  GtSample gt;
  // RTK-driven IMU bias calibration needs CM-LEVEL truth -- only consume
  // RTK-FIXED samples. If only dead-reckoned Atlas poses are available the
  // init machine will time out (rtk_init/fallback_timeout) and drop into
  // stationary calibration.
  if (!this->getGtPoseAt(stamp, gt) || !this->gtSampleIsRtkFixed(gt)) {
    // GT not yet available at this IMU stamp (e.g., IMU briefly ahead of buffer).
    // Don't error — just skip this sample.
    return false;
  }
  Eigen::Vector3f gt_p_in_base;
  Eigen::Quaternionf gt_q_in_base;
  Eigen::Vector3f gt_v_lin_base_body;
  Eigen::Vector3f gt_v_ang_base_body;
  if (!this->composeGtPoseInBase(gt, gt_p_in_base, gt_q_in_base) ||
      !this->composeGtTwistInBase(gt, gt_v_lin_base_body, gt_v_ang_base_body)) {
    // GT sample exists, but base<-gt_body TF is not cached yet.
    return false;
  }
  GtSample seed = gt;
  seed.p = gt_p_in_base;
  seed.q = gt_q_in_base;
  seed.v_lin_body = gt_v_lin_base_body;
  seed.v_ang_body = gt_v_ang_base_body;
  this->latest_rtk_seed_ = seed;
  this->has_latest_rtk_seed_ = true;

  // Body acceleration in world frame, from finite-differencing v_world across
  // consecutive paired samples. Skip the first sample (no derivative possible).
  const Eigen::Matrix3f R = seed.q.toRotationMatrix();
  const Eigen::Vector3f v_world = R * seed.v_lin_body;

  if (!this->has_prev_gt_for_accel_) {
    this->has_prev_gt_for_accel_ = true;
    this->prev_gt_stamp_ = stamp;
    this->prev_v_world_ = v_world;
    return false;
  }

  const double dt = stamp - this->prev_gt_stamp_;
  if (dt <= 1e-4) {
    // Sample too close in time — derivative would explode. Skip.
    return false;
  }
  const Eigen::Vector3f a_world = (v_world - this->prev_v_world_) / static_cast<float>(dt);
  this->prev_gt_stamp_ = stamp;
  this->prev_v_world_ = v_world;

  // Specific-force convention: at rest body-upright, the IMU reads ~(0,0,+g) in body
  // (confirmed by stationary-calibration gravity_dir output on this rig). Generalized
  // to moving: expected_accel_body = R^T * (a_world + (0,0,+g)). Sign of g is +,
  // not −, because the accelerometer measures proper acceleration (= inertial − g_world
  // = inertial + (0,0,+g) when world Z points up).
  const Eigen::Vector3f g_world(0.0f, 0.0f, +static_cast<float>(this->gravity_));
  const Eigen::Vector3f expected_accel_body = R.transpose() * (a_world + g_world);
  const Eigen::Vector3f expected_gyro_body = seed.v_ang_body;

  const Eigen::Vector3f gyro_res = measured_gyro - expected_gyro_body;
  const Eigen::Vector3f accel_res = measured_accel - expected_accel_body;

  this->rtk_gyro_bias_sum_ += gyro_res;
  this->rtk_accel_bias_sum_ += accel_res;
  this->rtk_gyro_bias_sq_sum_ += gyro_res.cwiseProduct(gyro_res);
  this->rtk_accel_bias_sq_sum_ += accel_res.cwiseProduct(accel_res);
  this->rtk_calib_count_++;

  const double elapsed = stamp - this->rtk_calib_start_stamp_;
  if (elapsed < this->rtk_calib_window_sec_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "IMU calibrating (RTK-driven)... %.1f/%.1fs (%d samples)",
                         elapsed, this->rtk_calib_window_sec_, this->rtk_calib_count_);
    return false;
  }
  if (this->rtk_calib_count_ < 2) {
    // Window expired but we got essentially no useful pairings (e.g., GT buffer
    // empty most of the window). Give up on RTK init and let the caller
    // decide — return false but signal via a warn.
    RCLCPP_WARN(this->get_logger(),
                "RTK init: window expired with only %d residual samples; cannot calibrate. "
                "Falling back to stationary path.", this->rtk_calib_count_);
    this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    this->imu_calib_start_stamp_ = stamp;
    return false;
  }

  const float n = static_cast<float>(this->rtk_calib_count_);
  const Eigen::Vector3f gyro_bias = this->rtk_gyro_bias_sum_ / n;
  const Eigen::Vector3f accel_bias = this->rtk_accel_bias_sum_ / n;

  // Sanity check on the averaged bias magnitudes. Noise variance is not a useful
  // signal here — finite-differencing GT velocity at IMU rate amplifies cm-level
  // GPS noise into ~10 m/s² of accel-residual stddev even on a stationary vehicle.
  // The mean averages that out cleanly, so we only reject if the averaged bias
  // itself is implausible. Typical biases on real IMUs: gyro <0.05 rad/s,
  // accel <0.3 m/s². Generous thresholds here so a moderately drifted IMU is
  // still accepted.
  if (gyro_bias.norm() > 1.0f || accel_bias.norm() > 5.0f) {
    RCLCPP_WARN(this->get_logger(),
                "RTK init: averaged bias magnitudes implausible (|gyro|=%.3f rad/s, "
                "|accel|=%.3f m/s^2); falling back to stationary calibration",
                gyro_bias.norm(), accel_bias.norm());
    this->init_phase_ = InitPhase::STATIONARY_CALIBRATING;
    this->imu_calib_start_stamp_ = stamp;
    this->rtk_calib_count_ = 0;
    this->rtk_gyro_bias_sum_.setZero();
    this->rtk_accel_bias_sum_.setZero();
    this->rtk_gyro_bias_sq_sum_.setZero();
    this->rtk_accel_bias_sq_sum_.setZero();
    return false;
  }

  // [REVIEW FIX 2026-07-08 P2] Apply biases; seed POSE state only when
  // nothing else has initialized the filter yet. Previously this always
  // re-seeded state/geo.prev_* from the latest RTK sample but left
  // current_pose / basePose / base_pose_stamp_ / prev_vel untouched:
  //   * use_odom_init=true (already initialized): the observer state jumped
  //     to the RTK sample while the GICP scan chain kept integrating from the
  //     old basePose — two briefly divergent pose chains.
  //   * use_odom_init=false: state was seeded but `initialized` was never
  //     set, so callbackPointCloud waited forever.
  // Now: bias-only once initialized; otherwise a FULL seed that mirrors
  // applyInitialPose()/maybeSnapPoseToGT() (current_pose, basePose,
  // base_pose_stamp_, prev_vel, observer state), publishing initialized=true
  // LAST so a concurrent scan callback never sees a half-written seed.
  const bool already_seeded = this->initialized.load();
  const Eigen::Vector3f v_seed_world =
      this->latest_rtk_seed_.q * this->latest_rtk_seed_.v_lin_body;
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.b.gyro = gyro_bias;
    this->state.b.accel = accel_bias;
    if (!already_seeded) {
      this->state.p = this->latest_rtk_seed_.p;
      this->state.q = this->latest_rtk_seed_.q;
      this->state.v.lin.b = this->latest_rtk_seed_.v_lin_body;
      this->state.v.lin.w = v_seed_world;
      this->state.v.ang.b = this->latest_rtk_seed_.v_ang_body;
      this->state.v.ang.w = this->latest_rtk_seed_.q * this->latest_rtk_seed_.v_ang_body;
      this->geo.prev_p = this->latest_rtk_seed_.p;
      this->geo.prev_q = this->latest_rtk_seed_.q;
      this->geo.prev_vel = v_seed_world;
      ++this->observer_epoch_;
      this->observer_pose_history_.clear();
      this->latency_compensated_output_.valid = false;
      // [REVIEW FIX 2026-07-08 P2] The state is a complete, valid estimate
      // (pose + velocities + biases): mark the observer initialized, exactly
      // as GT odom-init does. Otherwise the first accepted GICP scan entered
      // the first-time observer branch, which (previously) zeroed the freshly
      // calibrated biases and (still) discards the seeded velocities.
      this->geo.first_opt_done = true;
    }
    ++this->geo.update_seq;  // discard in-flight propagateState computations
  }
  if (!already_seeded) {
    Eigen::Matrix4f T_seed = Eigen::Matrix4f::Identity();
    T_seed.block<3, 3>(0, 0) = this->latest_rtk_seed_.q.toRotationMatrix();
    T_seed.block<3, 1>(0, 3) = this->latest_rtk_seed_.p;
    {
      std::lock_guard<std::mutex> lock(this->pose_mutex);
      this->current_pose = T_seed;
    }
    {
      std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);  // [P2 FIX 2026-07-09]
      this->basePose.p = this->latest_rtk_seed_.p;
      this->basePose.q = this->latest_rtk_seed_.q;
      this->base_pose_stamp_ = this->latest_rtk_seed_.stamp;
      this->prev_vel = v_seed_world;
      this->T_prior_velocity_ = v_seed_world;
    }
    this->initialized = true;  // publish only after ALL pose state is consistent
  }

  this->imu_calibrated_ = true;
  RCLCPP_INFO(this->get_logger(),
              "IMU calibrated (RTK-driven, %d samples, %.1fs, %s): "
              "gyro_bias=[%.4f,%.4f,%.4f] accel_bias=[%.3f,%.3f,%.3f] "
              "seed_pos=[%.2f,%.2f,%.2f] seed_v=[%.2f,%.2f,%.2f]m/s",
              this->rtk_calib_count_, elapsed,
              already_seeded ? "bias-only, pose chains already seeded"
                             : "full pose seed",
              gyro_bias.x(), gyro_bias.y(), gyro_bias.z(),
              accel_bias.x(), accel_bias.y(), accel_bias.z(),
              this->latest_rtk_seed_.p.x(), this->latest_rtk_seed_.p.y(), this->latest_rtk_seed_.p.z(),
              v_seed_world.x(), v_seed_world.y(), v_seed_world.z());
  return true;
}

bool gicp_localizer::GicpLocalizer::maybeSnapPoseToGT(
    const char* reason, bool force_absolute, bool immediate_after_sync_loss) {
  // DIAGNOSTIC: prove helper is being called. Remove once snap behavior verified.
  // [P3 FIX 2026-07-10] Demoted from unconditional INFO ("prove helper is
  // being called" diagnostic) — it fired on EVERY non-accepted scan, ~10
  // lines/s of noise interleaved with the SCAN DEBUG evidence during streaks.
  RCLCPP_DEBUG(this->get_logger(),
              "GT recovery: maybeSnapPoseToGT entered (enabled=%d streak=%d/%d immediate_sync=%d gt_received=%d cached=%d) reason='%s'",
              this->gt_recovery_enabled_,
              this->consecutive_failures_, this->gt_recovery_min_consecutive_failures_,
              immediate_after_sync_loss ? 1 : 0,
              this->gt_odom_received_.load() ? 1 : 0,
              // [P1 FIX 2026-07-09] atomic<bool> cannot be passed to a vararg
              // (deleted copy ctor -> build break); load and promote explicitly.
              this->gt_extrinsics_cached_.load() ? 1 : 0, reason);
  // Guards. Below-threshold guard is silent (frequent on every rejection until
  // streak builds up); the others log throttled info so a misconfiguration
  // doesn't silently disable recovery.
  if (!this->gt_recovery_enabled_) return false;
  if (!immediate_after_sync_loss &&
      this->consecutive_failures_ < this->gt_recovery_min_consecutive_failures_) {
    return false;
  }
  if (!this->gt_odom_received_.load()) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — no GT odom received yet (streak=%d)",
                this->consecutive_failures_);
    return false;
  }
  if (!this->gt_extrinsics_cached_) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — base→%s TF not cached yet (streak=%d)",
                this->gt_body_frame_.c_str(), this->consecutive_failures_);
    return false;
  }

  GtSample gt;
  // Log buffer state before the lookup so we can diagnose silent failures.
  size_t buf_size = 0;
  double buf_oldest = 0.0, buf_newest = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->gt_odom_mtx_);
    buf_size = this->gt_odom_buffer_.size();
    if (buf_size > 0) {
      buf_oldest = this->gt_odom_buffer_.front().stamp;
      buf_newest = this->gt_odom_buffer_.back().stamp;
    }
  }
  // [P1 FIX 2026-07-10i] ONE validity time for the whole snap: the estimate
  // the delta is measured against (current_pose = T_prior on the reject path
  // that calls us) is a MEDIAN-point-time pose (t_prior_stamp_). Querying GT
  // at the LiDAR HEADER stamp instead put ~half a sweep of REAL vehicle
  // motion inside the correction delta (~3 m at 60 m/s with a 50 ms offset),
  // biasing the live observer on every snap. Query GT at t_prior_stamp_ and
  // retain that stamp for basePose/velocity backfills below.
  const double snap_stamp =
      (this->t_prior_stamp_ > 0.0) ? this->t_prior_stamp_ : this->scan_stamp.seconds();
  bool got = this->getGtPoseAt(snap_stamp, gt);
  RCLCPP_INFO(this->get_logger(),
              "GT recovery: lookup snap_stamp=%.3f (median-time) got=%d buf=[size=%zu oldest=%.3f newest=%.3f] max_dt=%.3f",
              snap_stamp, got, buf_size, buf_oldest, buf_newest, this->gt_odom_max_dt_);
  if (!got) {
    RCLCPP_WARN(this->get_logger(),
                "GT recovery: deferring snap — no GT sample within max_dt=%.3fs of scan stamp %.3f (streak=%d)",
                this->gt_odom_max_dt_, this->scan_stamp.seconds(), this->consecutive_failures_);
    return false;
  }

  // Pose composition: bring gt sample from gt_body_frame into base_frame.
  // Shared with other GT consumers via composeGtPoseInBase.
  Eigen::Vector3f p_new;
  Eigen::Quaternionf q_new;
  if (!this->composeGtPoseInBase(gt, p_new, q_new)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "GT recovery: composeGtPoseInBase returned false "
                         "(extrinsic not cached). Deferring snap.");
    return false;
  }

  // A planar localizer must not import live odometry Z, but it also must not
  // freeze the cloud at the starting height on Laguna's elevation changes.
  // Recovery owns XY/yaw; internal Z comes from the static TTL elevation
  // profile plus the one-shot map datum calibrated at initialization.
  const bool planar_z_locked =
      this->gicp_dof_mode_ == "planar" &&
      this->track_z_hold_valid_.load(std::memory_order_acquire);
  float locked_z = planar_z_locked
                       ? static_cast<float>(this->track_z_hold_m_.load(
                             std::memory_order_relaxed))
                       : p_new.z();
  if (planar_z_locked && this->track_elevation_enabled_ &&
      this->track_constraint_ && this->track_z_offset_valid_) {
    Eigen::Matrix4f recovery_pose = Eigen::Matrix4f::Identity();
    recovery_pose.block<3, 3>(0, 0) = q_new.toRotationMatrix();
    recovery_pose.block<3, 1>(0, 3) = p_new;
    const auto elevation =
        this->track_constraint_->projectElevation(recovery_pose);
    if (elevation.valid) {
      locked_z = static_cast<float>(elevation.z + this->track_z_offset_m_);
    }
  }
  if (planar_z_locked) {
    p_new.z() = locked_z;
    this->track_z_hold_m_.store(static_cast<double>(locked_z),
                                std::memory_order_relaxed);
  }

  // Twist composition (P2#2): resolve the linear and angular sources
  // INDEPENDENTLY. Run 12/13 Atlas odom carried a valid linear twist but ZERO
  // angular twist (the adapter didn't populate twist.angular from the gyro),
  // and the old all-or-nothing "twist_is_zero" check passed the zeros through
  // — every one of the 1,860 snaps reset the angular-rate state to zero
  // mid-turn. Sources by priority:
  //   angular: GT twist -> live IMU gyro (bias-corrected under geo.mtx below;
  //            angular rate is rigid-body-invariant and the Atlas gyro IS the
  //            measured body rate in these axes) -> zero.
  //   linear:  GT twist -> finite difference of the bracketing GT poses
  //            (handles both an unpopulated twist and a true standstill
  //            uniformly) -> keep the current state velocity (never zero a
  //            moving vehicle's velocity: with the old behavior the next
  //            IMU prior integrates from v=0 and immediately re-fails).
  const Eigen::Matrix3f R_base_gtbody = this->T_base_gtbody_.block<3, 3>(0, 0);
  const Eigen::Vector3f t_base_gtbody = this->T_base_gtbody_.block<3, 1>(0, 3);
  const Eigen::Matrix3f R_gtbody_base = R_base_gtbody.transpose();
  const Eigen::Vector3f t_gtbody_base = -R_gtbody_base * t_base_gtbody;

  const bool gt_ang_valid = gt.v_ang_body.norm() >= 0.01f;
  const bool gt_lin_valid = gt.v_lin_body.norm() >= 0.05f;

  Eigen::Vector3f omega_base_body = Eigen::Vector3f::Zero();
  bool omega_from_imu = false;
  if (gt_ang_valid) {
    // [P3 FIX 2026-07-14] R_base_gtbody (not the transpose) rotates the gt_body
    // twist into base coords — see composeGtTwistInBase.
    omega_base_body = R_base_gtbody * gt.v_ang_body;
  } else {
    std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
    // The scan worker can lag the executor by seconds during a replay burst.
    // `imu_meas` is therefore the newest *future* sample, not necessarily the
    // angular velocity at snap_stamp.  Choose the nearest buffered sample at
    // the recovery time; the buffer is timestamp ordered newest -> oldest and
    // all entries are bias-corrected at ingestion.
    const ImuMeas* nearest = nullptr;
    double nearest_dt = std::numeric_limits<double>::infinity();
    for (const auto& sample : this->imu_buffer) {
      const double dt = std::abs(sample.stamp - snap_stamp);
      if (dt < nearest_dt) {
        nearest = &sample;
        nearest_dt = dt;
      }
    }
    if (nearest != nullptr && nearest_dt < 0.2) {
      omega_base_body = nearest->ang_vel;
      omega_from_imu = true;
    }
  }

  Eigen::Vector3f v_base_body = Eigen::Vector3f::Zero();
  bool lin_resolved = false;
  bool lin_from_fd = false;
  if (gt_lin_valid) {
    // [P3 FIX 2026-07-14] R_base_gtbody (not the transpose) — see composeGtTwistInBase.
    v_base_body = R_base_gtbody * (gt.v_lin_body + gt.v_ang_body.cross(t_gtbody_base));
    lin_resolved = true;
  } else {
    Eigen::Vector3f v_fd_world;
    if (this->getGtFiniteDiffVelWorld(snap_stamp, v_fd_world)) {
      v_base_body = q_new.conjugate() * v_fd_world;
      lin_resolved = true;
      lin_from_fd = true;
    }
  }

  static bool warned_twist_sources = false;
  if (!warned_twist_sources && (!gt_ang_valid || !gt_lin_valid)) {
    warned_twist_sources = true;
    RCLCPP_INFO(this->get_logger(),
                "GT recovery: GT twist partially unpopulated (lin=%.3f m/s%s, ang=%.3f rad/s%s); "
                "backfilling angular from %s and linear from %s",
                gt.v_lin_body.norm(), gt_lin_valid ? "" : " INVALID",
                gt.v_ang_body.norm(), gt_ang_valid ? "" : " INVALID",
                gt_ang_valid ? "GT" : (omega_from_imu ? "time-matched IMU gyro" : "zero"),
                gt_lin_valid ? "GT" : (lin_from_fd ? "GT pose finite-difference" : "current state"));
  }

  // Apply state. Caller (performLocalization) already holds pose_mutex (line 1768),
  // so we MUST NOT re-acquire it here — std::mutex is non-recursive and that would
  // deadlock the entire scan callback. geo.mtx, however, is taken in narrow scopes
  // by performLocalization, never held across this call site, so locking it here
  // is correct.
  // [P2 FIX 2026-07-10d] DELTA-FORM observer snap. The GT sample is at
  // scan_stamp, but the observer has been IMU-propagated PAST it by
  // callback/solver latency (deskew + align: 20-100+ ms). The old absolute
  // overwrite REWOUND the live observer to the stale scan-time pose — metres
  // backward at racing speed — and discarded every IMU measurement since the
  // scan. Mirror updateState's delta form instead: measure the correction AT
  // scan time against the scan chain's estimate (current_pose == T_prior on
  // the reject path that calls us; the observer dead-reckons from the same
  // anchor during a failure streak, so the deltas coincide), then apply that
  // SE(3) delta to the CURRENT observer state — the scan-time error snaps
  // away while the propagation since the scan is preserved.
  const Eigen::Matrix4f T_est_scan = this->current_pose;
  Eigen::Matrix4f T_gt_scan = Eigen::Matrix4f::Identity();
  T_gt_scan.block<3, 3>(0, 0) = q_new.toRotationMatrix();
  T_gt_scan.block<3, 1>(0, 3) = p_new;
  const bool snap_estimate_finite = matrixFinite(T_est_scan);
  const bool apply_delta = !force_absolute && snap_estimate_finite;
  const Eigen::Matrix4f T_corr =
      snap_estimate_finite
          ? Eigen::Matrix4f(T_gt_scan * T_est_scan.inverse())
          : T_gt_scan;

  this->current_pose = T_gt_scan;  // scan-chain pose stays a scan-time quantity
  Eigen::Vector3f v_base_world;
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    // (P3: no bias subtraction here — imu_meas is bias-corrected at buffering;
    // omega_from_imu is noted only for the source log above.)
    (void)omega_from_imu;
    if (!lin_resolved) {
      // Last resort: preserve the IMU-propagated velocity through the snap.
      v_base_body = this->state.q.conjugate() * this->state.v.lin.w;
    }
    // Scan-time world velocity: seeds the NEXT scan's integration (prev_vel,
    // paired with base_pose_stamp_ = scan_stamp below).
    v_base_world = q_new * v_base_body;
    if (planar_z_locked) {
      v_base_world.z() = 0.0f;
      v_base_body = q_new.conjugate() * v_base_world;
    }

    Eigen::Quaternionf q_state_new;
    Eigen::Vector3f p_state_new;
    if (apply_delta) {
      Eigen::Quaternionf q_corr(Eigen::Matrix3f(T_corr.block<3, 3>(0, 0)));
      q_corr.normalize();
      q_state_new = (q_corr * this->state.q).normalized();
      p_state_new = T_corr.block<3, 3>(0, 0) * this->state.p + T_corr.block<3, 1>(0, 3);
    } else {
      // Catastrophic/wrong-lock branch or degenerate pre-snap estimate:
      // overwrite the observer absolutely instead of preserving a bad basin.
      q_state_new = q_new;
      p_state_new = p_new;
    }
    // Velocities: GT body rates re-expressed with the CORRECTED current
    // attitude (body rates are ~constant over the latency window).
    if (planar_z_locked) {
      p_state_new.z() = locked_z;
    }
    Eigen::Vector3f v_world_now = q_state_new * v_base_body;
    if (planar_z_locked) {
      v_world_now.z() = 0.0f;
    }
    const Eigen::Vector3f omega_world_now = q_state_new * omega_base_body;
    this->state.p = p_state_new;
    this->state.q = q_state_new;
    this->state.v.lin.b = q_state_new.conjugate() * v_world_now;
    this->state.v.lin.w = v_world_now;
    this->state.v.ang.b = omega_base_body;
    this->state.v.ang.w = omega_world_now;
    // state.b.gyro and state.b.accel intentionally preserved.
    this->geo.prev_p = p_state_new;
    this->geo.prev_q = q_state_new;
    this->geo.prev_vel = v_world_now;
    ++this->observer_epoch_;
    this->observer_pose_history_.clear();
    this->latency_compensated_output_.valid = false;
    ++this->geo.update_seq;  // discard any in-flight propagateState computations
  }
  RCLCPP_INFO(this->get_logger(),
              "GT recovery: %s snap — correction |t|=%.2f m |rot|=%.2f deg applied to the "
              "LIVE observer state%s",
              apply_delta ? "delta-form" : "absolute",
              T_corr.block<3, 1>(0, 3).norm(),
              rotationDistanceDeg(Eigen::Matrix4f::Identity(), T_corr),
              planar_z_locked
                  ? " (planar Z preserved)"
                  : (apply_delta
                         ? ""
                         : (force_absolute
                                ? " (forced by catastrophic/wrong-lock rejection)"
                                : " (pre-snap estimate non-finite)")));
  {
    // [P2 FIX 2026-07-09] Seed writes under the owner lock (pose -> seed:
    // maybeSnapPoseToGT runs inside performLocalization's pose_mutex scope).
    std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
    this->basePose.p = p_new;
    this->basePose.q = q_new;
    // [P1 FIX 2026-07-10i] The snap pose is the GT sample at snap_stamp
    // (median point time) — matching the accept/reject stamps and the
    // estimate the delta was formed against.
    this->base_pose_stamp_ = snap_stamp;
    this->t_prior_stamp_ = snap_stamp;
    this->prev_vel = v_base_world;
    this->T_prior_velocity_ = v_base_world;
  }

  RCLCPP_WARN(this->get_logger(),
              "Localization: ⟳ snapped pose to GT (%s%s after %d consecutive non-accepts) — "
              "pose=[%.2f,%.2f,%.2f] v=[%.2f,%.2f,%.2f]m/s ω=[%.2f,%.2f,%.2f]rad/s | gt_body=%s",
              reason, immediate_after_sync_loss ? "; immediate strict-sync recovery" : "",
              this->consecutive_failures_,
              p_new.x(), p_new.y(), p_new.z(),
              v_base_world.x(), v_base_world.y(), v_base_world.z(),
              omega_base_body.x(), omega_base_body.y(), omega_base_body.z(),
              this->gt_body_frame_.c_str());

  this->consecutive_failures_ = 0;
  this->failure_streak_had_timeout_ = false;
  this->previous_failure_track_along_ = false;
  // A GT snap restores a known-good pose, so restart the dead-reckon clock (P3).
  this->last_accepted_scan_stamp_ = this->scan_stamp.seconds();
  // [P3 FIX 2026-07-10] Post-snap gate/covariance bookkeeping: without these,
  // the next scan's dt-scaled jump/yaw budgets were computed against the
  // PRE-streak accepted scan (scan_dt clamped to max -> loosest gates against
  // a now GT-fresh prior), and the published covariance kept describing a
  // fitness from before the streak.
  this->last_gicp_pose_ = this->current_pose;    // = GT pose at snap_stamp (median time)
  this->last_gicp_stamp_ = this->scan_stamp;     // tight dt budget vs fresh prior
  this->last_gicp_valid_ = true;
  this->last_accepted_fitness_score_ = -1.0;     // covariance -> base sigma floor
  return true;
}
