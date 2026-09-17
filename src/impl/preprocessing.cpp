#include "gicp_interface/detail/localizer_utils.hpp"

#include <pcl/filters/voxel_grid.h>
#include "gicp_interface/deskew_time_knots.hpp"

using gicp_localizer::detail::logLuminarTimestampStats;
using gicp_localizer::detail::luminarPointTimestampNs;
using gicp_localizer::detail::matrixFinite;

void gicp_localizer::GicpLocalizer::applyWorldYawStepToSeedAndObserver(
    double step_deg) {
  if (!std::isfinite(step_deg) || step_deg == 0.0)
    return;

  const Eigen::Quaternionf rz(Eigen::AngleAxisf(
      static_cast<float>(step_deg * M_PI / 180.0), Eigen::Vector3f::UnitZ()));
  this->basePose.q = (rz * this->basePose.q).normalized();
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    this->state.q = (rz * this->state.q).normalized();
    this->state.v.lin.w = rz * this->state.v.lin.w;
    this->state.v.ang.w = rz * this->state.v.ang.w;
    this->geo.prev_q = (rz * this->geo.prev_q).normalized();
    this->geo.prev_vel = rz * this->geo.prev_vel;
    const double newest_stamp = this->observer_pose_history_.empty()
                                    ? 0.0
                                    : this->observer_pose_history_.back().stamp;
    ++this->observer_epoch_;
    this->observer_pose_history_.clear();
    if (newest_stamp > 0.0) {
      this->observer_pose_history_.push_back(
          {newest_stamp, this->state.p, this->state.q, this->observer_epoch_});
    }
    this->latency_compensated_output_.valid = false;
    this->latency_compensated_output_.status = "observer_epoch_changed";
    ++this->geo.update_seq;
  }
  this->prev_vel = rz * this->prev_vel;
  this->T_prior_velocity_ = rz * this->T_prior_velocity_;
}

bool gicp_localizer::GicpLocalizer::getImuHeadingAt(
    double stamp, Eigen::Quaternionf &orientation_world_base,
    double &yaw_variance, double &time_error_s) {
  std::lock_guard<std::mutex> lock(this->mtx_imu);
  const ImuMeas *newer = nullptr;
  const ImuMeas *older = nullptr;

  // imu_buffer is newest -> oldest. Skip messages whose orientation is
  // explicitly unavailable, retaining the nearest valid bracketing pair.
  for (const auto &sample : this->imu_buffer) {
    if (!sample.orientation_valid)
      continue;
    if (sample.stamp >= stamp) {
      newer = &sample;
      continue;
    }
    older = &sample;
    break;
  }

  if (newer != nullptr && older != nullptr) {
    const double span = newer->stamp - older->stamp;
    if (!(span > 0.0) || !std::isfinite(span))
      return false;
    time_error_s = std::min(newer->stamp - stamp, stamp - older->stamp);
    if (!std::isfinite(time_error_s) || time_error_s < 0.0 ||
        time_error_s > this->imu_heading_prior_max_time_error_s_) {
      return false;
    }
    const float alpha =
        static_cast<float>(std::clamp((stamp - older->stamp) / span, 0.0, 1.0));
    orientation_world_base = older->orientation_world_base
                                 .slerp(alpha, newer->orientation_world_base)
                                 .normalized();
    yaw_variance = std::max(older->orientation_yaw_variance,
                            newer->orientation_yaw_variance);
    return true;
  }

  const ImuMeas *nearest = newer != nullptr ? newer : older;
  if (nearest == nullptr)
    return false;
  time_error_s = std::abs(nearest->stamp - stamp);
  if (!std::isfinite(time_error_s) ||
      time_error_s > this->imu_heading_prior_max_time_error_s_) {
    return false;
  }
  orientation_world_base = nearest->orientation_world_base;
  yaw_variance = nearest->orientation_yaw_variance;
  return true;
}

void gicp_localizer::GicpLocalizer::applyImuHeadingPriorToBasePose() {
  this->last_ins_yaw_diff_deg_ = std::numeric_limits<double>::quiet_NaN();
  if (!this->imu_heading_prior_enable_ || this->prev_scan_stamp <= 0.0)
    return;

  const double seed_stamp = (this->base_pose_stamp_ > 0.0)
                                ? this->base_pose_stamp_
                                : this->prev_scan_stamp;
  Eigen::Quaternionf imu_q;
  double yaw_variance = -1.0;
  double time_error_s = std::numeric_limits<double>::infinity();
  if (!this->getImuHeadingAt(seed_stamp, imu_q, yaw_variance, time_error_s)) {
    return;
  }

  if (!std::isfinite(yaw_variance) || yaw_variance < 0.0)
    return;
  if (this->imu_heading_prior_max_yaw_sigma_deg_ > 0.0) {
    const double yaw_sigma_deg = std::sqrt(yaw_variance) * 180.0 / M_PI;
    if (!std::isfinite(yaw_sigma_deg) ||
        yaw_sigma_deg > this->imu_heading_prior_max_yaw_sigma_deg_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "IMU heading prior: yaw sigma %.2f deg > %.2f deg; skipped",
          yaw_sigma_deg, this->imu_heading_prior_max_yaw_sigma_deg_);
      return;
    }
  }

  const Eigen::Matrix3d R_seed =
      this->basePose.q.normalized().toRotationMatrix().cast<double>();
  const Eigen::Matrix3d R_imu =
      imu_q.normalized().toRotationMatrix().cast<double>();
  const Eigen::AngleAxisd aa(R_imu * R_seed.transpose());
  const double dyaw_deg = (aa.angle() * aa.axis()).z() * 180.0 / M_PI;
  this->last_ins_yaw_diff_deg_ = dyaw_deg;
  if (!std::isfinite(dyaw_deg) ||
      std::abs(dyaw_deg) > this->imu_heading_prior_sanity_max_yaw_deg_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "IMU heading prior: |IMU-vs-prior yaw|=%.1f deg exceeds %.1f deg; "
        "skipped (check orientation world frame)",
        dyaw_deg, this->imu_heading_prior_sanity_max_yaw_deg_);
    return;
  }

  double step_deg = this->imu_heading_prior_yaw_blend_ * dyaw_deg;
  step_deg = std::clamp(step_deg, -this->imu_heading_prior_max_yaw_step_deg_,
                        this->imu_heading_prior_max_yaw_step_deg_);
  if (step_deg != 0.0) {
    const Eigen::Quaternionf rz(Eigen::AngleAxisf(
        static_cast<float>(step_deg * M_PI / 180.0), Eigen::Vector3f::UnitZ()));
    // Correct the scan-time seed only. The live observer/history must remain
    // untouched until an accepted GICP measurement reaches updateState();
    // clearing it here on every scan would disable the short latency bridge.
    this->basePose.q = (rz * this->basePose.q).normalized();
    this->prev_vel = rz * this->prev_vel;
    this->T_prior_velocity_ = rz * this->T_prior_velocity_;
  }
}

void gicp_localizer::GicpLocalizer::applyInsHeadingPriorToBasePose() {
  this->last_ins_yaw_diff_deg_ = std::numeric_limits<double>::quiet_NaN();
  if (!this->ins_prior_enable_ || !this->gt_odom_enabled_ ||
      !this->gt_odom_received_.load() || this->prev_scan_stamp <= 0.0) {
    return;
  }
  // [REVIEW FIX 2026-07-08] Query the INS at the time basePose is actually
  // valid at. basePose is the pose at the MEDIAN POINT TIME of the previous
  // scan (frames[median_pt_index] / the accepted candidate), while
  // prev_scan_stamp is the scan HEADER time -- typically ~half a sweep
  // (~50 ms) earlier. Comparing the median-time seed attitude against the
  // header-time INS attitude injected a yaw-rate-proportional bias on turns
  // (50 ms * 30 deg/s = 1.5 deg), which the blend then pulled INTO the seed.
  // base_pose_stamp_ is maintained at every basePose write site; fall back to
  // prev_scan_stamp only if it was never set (pre-first-scan states).
  const double seed_stamp =
      (this->base_pose_stamp_ > 0.0) ? this->base_pose_stamp_ : this->prev_scan_stamp;
  GtSample ins;
  if (!this->getGtPoseAt(seed_stamp, ins)) return;
  if (this->ins_prior_require_rtk_ && !this->gtSampleIsRtkFixed(ins)) return;
  // [REVIEW FIX 2026-07-08] Heading-quality gate, mirroring GLIM gnss_global's
  // orientation_prior_max_yaw_sigma_deg. The position RTK gate above says
  // nothing about dual-antenna heading health: Atlas can be position-FIXED
  // while the heading solution is degraded (baseline outage, single-antenna
  // fallback), and a 0.25 blend at 2 deg/scan would happily steer the seed
  // toward that degraded heading. Yaw variance (rad^2) arrives per-sample in
  // pose.covariance[35] via the adapter; unpopulated (<=0) passes for compat
  // with GT sources that do not fill it (same convention as GLIM).
  if (this->ins_prior_max_yaw_sigma_deg_ > 0.0 && ins.cov_yaw > 0.0) {
    const double yaw_sigma_deg = std::sqrt(ins.cov_yaw) * 180.0 / M_PI;
    if (yaw_sigma_deg > this->ins_prior_max_yaw_sigma_deg_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "INS prior: heading quality gate — yaw sigma %.2f deg > %.2f deg, "
                           "prior NOT applied (position RTK may still be FIXED).",
                           yaw_sigma_deg, this->ins_prior_max_yaw_sigma_deg_);
      return;
    }
  }

  Eigen::Vector3f ins_p;
  Eigen::Quaternionf ins_q;
  if (!this->composeGtPoseInBase(ins, ins_p, ins_q)) {
    // [P3 FIX 2026-07-10] SKIP the prior instead of falling back to the raw
    // gt-body attitude: treating gt-body as base-frame steered a constant
    // frame offset INTO the seed at up to 2 deg/scan whenever the extrinsic
    // was uncached (TF missing in replay). Odom-init already defers on the
    // same condition.
    return;
  }

  // World-frame yaw difference: INS attitude vs the integration seed.
  const Eigen::Matrix3d R_seed = this->basePose.q.normalized().toRotationMatrix().cast<double>();
  const Eigen::Matrix3d R_ins = ins_q.normalized().toRotationMatrix().cast<double>();
  const Eigen::AngleAxisd aa(R_ins * R_seed.transpose());
  const double dyaw_deg = (aa.angle() * aa.axis()).z() * 180.0 / M_PI;
  this->last_ins_yaw_diff_deg_ = dyaw_deg;

  if (std::abs(dyaw_deg) > this->ins_prior_sanity_max_yaw_deg_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "INS prior: |INS-vs-prior yaw|=%.1f deg exceeds sanity %.1f deg — NOT "
                         "applied. Check map/ENU yaw alignment or INS attitude health.",
                         dyaw_deg, this->ins_prior_sanity_max_yaw_deg_);
    return;
  }

  double step_deg = this->ins_prior_yaw_blend_ * dyaw_deg;
  step_deg = std::clamp(step_deg, -this->ins_prior_max_yaw_step_deg_,
                        this->ins_prior_max_yaw_step_deg_);
  this->applyWorldYawStepToSeedAndObserver(step_deg);
  // Optional position blend (default 0 = OFF). This is a deliberate external
  // INS fusion mode and should be evaluated on held-out segments.
  if (this->ins_prior_pos_blend_ > 0.0) {
    const Eigen::Vector3f dp = static_cast<float>(this->ins_prior_pos_blend_) *
                               (ins_p - this->basePose.p);
    this->basePose.p += dp;
    // [REVIEW FIX 2026-07-08] Mirror the yaw handling: shift the IMU-rate
    // observer state by the SAME translation so the delta-form observer sees
    // a consistent correction. Without this, a GICP result that merely
    // confirms the shifted prior yields an identity delta and the IMU-rate
    // odometry (propagateState from state.p) never inherits the position
    // pull — scan-time and IMU-rate state split again. Velocities and
    // attitude are invariant under a pure translation.
    {
      std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
      this->state.p += dp;
      this->geo.prev_p += dp;
      const double newest_stamp = this->observer_pose_history_.empty()
                                      ? 0.0
                                      : this->observer_pose_history_.back().stamp;
      ++this->observer_epoch_;
      this->observer_pose_history_.clear();
      if (newest_stamp > 0.0) {
        this->observer_pose_history_.push_back(
            {newest_stamp, this->state.p, this->state.q,
             this->observer_epoch_});
      }
      this->latency_compensated_output_.valid = false;
      this->latency_compensated_output_.status = "observer_epoch_changed";
      ++this->geo.update_seq;  // discard any in-flight propagateState computations
    }
  }
}

bool gicp_localizer::GicpLocalizer::deskewPointcloud() {

  const auto deskew_begin = std::chrono::steady_clock::now();
  this->last_deskew_setup_ms_ = 0.0;
  this->last_deskew_imu_wait_ms_ = 0.0;
  this->last_deskew_imu_integrate_ms_ = 0.0;
  this->last_deskew_transform_ms_ = 0.0;

  // [P2 FIX 2026-07-09] Hold the seed owner lock for the whole deskew phase:
  // basePose/base_pose_stamp_/prev_vel are read (and, via the INS prior,
  // written) throughout, and the reinit writers (applyInitialPose, param
  // pose, RTK full seed) run on other threads. Cost: contended only during a
  // reinit. Lock order: seed -> {geo, imu}; no holder of pose/geo ever takes
  // seed, so no cycle.
  std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);

  // REVIEW FIX: reset the per-frame sweep-span diagnostic up front so early
  // returns (deskew off, unsupported sensor, empty IMU buffer, ...) publish
  // -1 instead of the previous frame's stale span.
  this->last_scan_time_span_s_ = -1.0;

  // Absolute heading prior — must run BEFORE any IMU integration or cloud
  // placement so the whole prior chain is consistent. The IMU orientation
  // source takes precedence when explicitly enabled; sources never stack.
  if (this->imu_heading_prior_enable_) {
    this->applyImuHeadingPriorToBasePose();
  } else {
    this->applyInsHeadingPriorToBasePose();
  }
  // Default to the velocity paired with basePose. Successful IMU integration
  // below overwrites this with the prediction at T_prior's exact timestamp.
  this->T_prior_velocity_ = this->prev_vel;

  // [P3 FIX 2026-07-14] Default validity time for this scan's T_prior: the time
  // basePose is ACTUALLY valid at (its previous median point time,
  // base_pose_stamp_). Every fallback branch below sets T_prior =
  // basePoseMatrix() — the UN-ADVANCED basePose — so labeling it with the
  // current header time (the old default) claimed motion that never happened:
  // a later reject then discarded the median→header interval and mis-timed the
  // INS-prior GT query on the next scan. Branches that genuinely advance the
  // pose override this with their real time (the deskew-off integration-success
  // branch → its integrated stamp; the main deskew path → the median point
  // time). Fall back to the header only before base_pose_stamp_ is ever set.
  this->t_prior_stamp_ =
      (this->base_pose_stamp_ > 0.0) ? this->base_pose_stamp_ : this->scan_stamp.seconds();

  // [REVIEW FIX 2026-07-08 P2] Default: cloud NOT world-frame. Only branches
  // that actually transform/place points into the world set this true.
  // performLocalization() keys its seed/composition on this flag, not deskew_.
  this->scan_in_world_frame_ = false;

  if (!this->deskew_ || !this->first_imu_received) {
    this->current_scan = this->original_scan;

    // Even without per-point deskewing, integrate IMU to get a motion-predicted
    // T_prior so GICP starts from an IMU-advanced pose instead of the stale last result.
    if (this->first_imu_received && this->prev_scan_stamp > 0.0) {
      std::vector<double> single_ts = {this->scan_stamp.seconds()};
      {
        double latest_imu = 0.0;
        {
          std::lock_guard<std::mutex> lock(this->mtx_imu);
          if (!this->imu_buffer.empty()) latest_imu = this->imu_buffer.front().stamp;
        }
        if (latest_imu > 0.0 && single_ts[0] > latest_imu)
          single_ts[0] = latest_imu;
      }
      // [P1 FIX 2026-07-10] same seed-time consistency as the main path.
      const double seed_time_off =
          (this->base_pose_stamp_ > 0.0) ? this->base_pose_stamp_ : this->prev_scan_stamp;
      std::vector<Eigen::Vector3f> velocities;
      auto frames = this->integrateImu(seed_time_off, this->basePose.q,
                                        this->basePose.p, this->prev_vel, single_ts,
                                        &velocities);
      if (frames.size() == 1 && velocities.size() == 1 &&
          matrixFinite(frames[0]) && velocities[0].allFinite()) {
        this->T_prior = frames[0];
        this->T_prior_velocity_ = velocities[0];
        // [P3 FIX 2026-07-14] This branch DID advance the pose (integrated to
        // the clamped header time single_ts[0]); label t_prior_stamp_ with that
        // real time, overriding the not-advanced default set above.
        this->t_prior_stamp_ = single_ts[0];
      } else {
        this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
      }
    } else {
      this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
    }

    this->prev_scan_stamp = this->scan_stamp.seconds();
    return true;
  }

  pcl::PointCloud<PointType>::Ptr deskewed_scan_ =
      std::make_shared<pcl::PointCloud<PointType>>(*this->original_scan);

  // Individual point timestamps should be relative to this time
  double sweep_ref_time = this->scan_stamp.seconds();

  // Decode each point timestamp once. The old implementation fully sorted all
  // ~65k points, then asked integrateImu() to materialize a 4x4 pose for every
  // unique ray timestamp. The new path keeps point order and later integrates
  // only a compact fixed-time trajectory grid.
  std::function<double(const PointType&)> extract_point_time_from_point;
  bool deskew_time_ready = false;

  if (this->sensor == gicp_localizer::SensorType::OUSTER) {
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) {
      return sweep_ref_time + static_cast<double>(pt.t) * 1e-9;
    };
    deskew_time_ready = true;
  } else if (this->sensor == gicp_localizer::SensorType::VELODYNE) {
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) {
      return sweep_ref_time + static_cast<double>(pt.time);
    };
    deskew_time_ready = true;
  } else if (this->sensor == gicp_localizer::SensorType::HESAI) {
    extract_point_time_from_point = [](const PointType& pt) { return pt.timestamp; };
    deskew_time_ready = true;
  } else if (this->sensor == gicp_localizer::SensorType::LIVOX) {
    extract_point_time_from_point = [](const PointType& pt) { return pt.timestamp * 1e-9; };
    deskew_time_ready = true;
  } else if (this->sensor == gicp_localizer::SensorType::LUMINAR &&
             this->luminar_scan_time_is_epoch_ns_) {
    // Per-point value is absolute PTP epoch ns (driver reconstruction of the
    // packet-header 48-bit seconds + per-ray 32-bit sub-second nanoseconds;
    // see Luminar Iris Data Output Specification v1.3.0 §2.1 and §2.2/§2.6.3).
    // We deskew on the relative offset (ts - anchor) anchored at the header
    // stamp, so the absolute epoch reference cancels. NOTE: this relies on the
    // driver supplying full epoch ns; a bare 32-bit ns field (sub-second, wraps
    // every 1 s) would make (ts - anchor) jump across a second boundary and
    // corrupt deskew for scans that straddle the rollover.
    //
    // ANCHOR: for a multi-LiDAR merged sweep, anchor on the PRIMARY scan's first
    // timestamp (captured in mergeAuxClouds() BEFORE aux append), NOT the global
    // merged minimum. An aux scan that began before the primary carries smaller
    // epoch timestamps; anchoring at the global min would map that aux point to
    // the header stamp (sweep_ref_time) and shift the entire merged sweep late.
    // Aux points earlier than the primary anchor therefore get correctly NEGATIVE
    // offsets -- which requires SIGNED subtraction below (uint64 underflow
    // otherwise). For the single-primary path we fall back to the global min,
    // which equals the primary min, so behavior is unchanged.
    uint64_t anchor_ts;
    if (this->luminar_primary_min_ts_valid_) {
      anchor_ts = this->luminar_primary_min_ts_ns_;
    } else {
      anchor_ts = std::numeric_limits<uint64_t>::max();
      for (const auto& pt : this->original_scan->points) {
        // [P3 FIX 2026-07-14] Skip the ts==0 "no valid time" sentinel so one
        // zero-stamped point cannot anchor the whole sweep at epoch 0.
        const uint64_t ts = luminarPointTimestampNs(pt);
        if (ts == 0) continue;
        anchor_ts = std::min(anchor_ts, ts);
      }
    }
    const uint64_t min_ts_captured = anchor_ts;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Luminar scan: anchor_ts=%lu ns (%s), sweep_ref=%.3f s", min_ts_captured,
                         this->luminar_primary_min_ts_valid_ ? "primary" : "global", sweep_ref_time);
    extract_point_time_from_point = [&sweep_ref_time, min_ts_captured](const PointType& pt) {
      const uint64_t ts = luminarPointTimestampNs(pt);
      // Signed difference: aux points earlier than the primary anchor are valid
      // and must produce negative offsets (epoch ns fits in int64_t).
      return sweep_ref_time + static_cast<double>(static_cast<int64_t>(ts) - static_cast<int64_t>(min_ts_captured)) * 1e-9;
    };
    deskew_time_ready = true;
  } else if (this->sensor == gicp_localizer::SensorType::LUMINAR) {
    // Laguna's decoder publishes FLOAT64 seconds since the start of each
    // sweep. mergeAuxClouds() rebases auxiliary values by
    // (T_aux_header - T_primary_header), so every merged point is already
    // expressed relative to the primary header.
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) {
      return sweep_ref_time + pt.timestamp;
    };
    deskew_time_ready = true;
  }

  if (!deskew_time_ready) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Unsupported localization/sensor_type for deskew; using scan without motion "
                         "compensation");
    // [REVIEW FIX 2026-07-08 P2] Sensor-frame cloud (scan_in_world_frame_
    // stays false) and a VALID prior for this scan — previously T_prior kept
    // its stale value from the last scan here.
    this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
    this->current_scan = this->original_scan;
    this->prev_scan_stamp = this->scan_stamp.seconds();
    return true;
  }

  std::vector<double> point_timestamps;
  point_timestamps.reserve(deskewed_scan_->points.size());
  for (const auto& point : deskewed_scan_->points) {
    point_timestamps.push_back(extract_point_time_from_point(point));
  }
  const DeskewTimeKnots trajectory = buildDeskewTimeKnots(
      point_timestamps, this->deskew_knot_interval_s_);
  this->last_deskew_setup_ms_ =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - deskew_begin)
          .count();

  if (this->sensor == gicp_localizer::SensorType::LUMINAR && this->verbose_ && !deskewed_scan_->points.empty()) {
    logLuminarTimestampStats(
        deskewed_scan_->points.size(), *deskewed_scan_, trajectory.times.size(),
        this->luminar_scan_time_is_epoch_ns_);
  }

  if (!trajectory.valid()) {
    RCLCPP_WARN(this->get_logger(), "No timestamps extracted from point cloud, skipping deskewing");
    this->last_scan_time_span_s_ = -1.0;
    // [REVIEW FIX 2026-07-08 P2] Sensor-frame cloud (scan_in_world_frame_
    // stays false); refresh T_prior and prev_scan_stamp (both previously left
    // stale in this branch).
    this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
    this->current_scan = this->original_scan;
    this->prev_scan_stamp = this->scan_stamp.seconds();
    return true;
  }

  // P4#3: per-frame sweep time span of the (merged) cloud. A healthy 3-LiDAR
  // merge spans ~1 sweep period; a much larger span means a badly-offset aux
  // got rebased far from the primary and is being deskewed across a long arc.
  this->last_scan_time_span_s_ =
      trajectory.times.back() - trajectory.times.front();

  // A Luminar sweep that collapses to a single unique timestamp means every point
  // shares one time, so deskew degenerates to a rigid transform (no motion
  // compensation). This is the symptom of a wrong per-point time encoding (e.g.
  // global_shutter/collapsed times) -- warn so the operator can fix the source.
  if (this->sensor == gicp_localizer::SensorType::LUMINAR && this->deskew_ && trajectory.times.size() == 1) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Luminar deskew collapsed to a single timestamp (%zu points share one time); "
                         "deskew reduced to a rigid transform. Check the per-point time encoding.",
                         deskewed_scan_->points.size());
  }

  const size_t median_knot_index = trajectory.measurement_index;

  // Don't process scans on first iteration
  if (this->prev_scan_stamp == 0.0) {
    this->prev_scan_stamp = this->scan_stamp.seconds();
    this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
    // Although the seed pose has not been IMU-advanced on this first frame,
    // the GICP candidate produced from the placed cloud is a measurement at
    // this scan's median point time. Label the candidate accordingly so an
    // accepted first scan advances base_pose_stamp_ instead of pinning every
    // subsequent integration request to the pre-replay GT seed timestamp.
    this->t_prior_stamp_ = trajectory.measurement_stamp;
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    this->scan_in_world_frame_ = true;
    return true;
  }

  // Do not make deskew depend on callback scheduling. The old code clamped
  // every point newer than imu_buffer.front() to that sample. In replay, the
  // same LiDAR sweep could therefore be clamped to e.g. t=...581 or ...591
  // depending only on whether one extra IMU callback ran before this worker.
  // That changed the median-time prior, voxel membership and ultimately the
  // GICP accept/reject path. Wait for a real IMU bracket through the final
  // point time instead. If it does not arrive within the bounded live-latency
  // budget, fail closed and drop this scan; a partial deskew is not a valid
  // deterministic fallback.
  const double required_latest_imu_stamp = trajectory.times.back();
  double latest_imu_stamp = -1.0;
  const auto imu_wait_start = std::chrono::steady_clock::now();
  {
    std::unique_lock<std::mutex> imu_lock(this->mtx_imu);
    const auto imu_covers_scan = [this, required_latest_imu_stamp] {
      return !this->imu_buffer.empty() &&
             this->imu_buffer.front().stamp >= required_latest_imu_stamp;
    };
    const bool ready =
        imu_covers_scan() ||
        this->imu_cv_.wait_for(
            imu_lock,
            std::chrono::duration<double>(this->future_imu_wait_timeout_s_),
            imu_covers_scan);
    if (!this->imu_buffer.empty()) {
      latest_imu_stamp = this->imu_buffer.front().stamp;
    }
    this->last_deskew_imu_wait_ms_ =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - imu_wait_start)
            .count();
    if (!ready) {
      ++this->future_imu_timeout_dropped_;
      RCLCPP_ERROR(
          this->get_logger(),
          "Deskew dropped scan %.6f: newest IMU %.6f does not cover final "
          "point %.6f after %.0fms wait (future_imu_timeout_dropped=%lu)",
          this->scan_stamp.seconds(), latest_imu_stamp,
          required_latest_imu_stamp,
          1e3 * this->future_imu_wait_timeout_s_,
          static_cast<unsigned long>(this->future_imu_timeout_dropped_));
      return false;
    }
  }

  // Check if we have sufficient IMU history
  // We need IMU data from BEFORE prev_scan_stamp to integrate
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (this->imu_buffer.empty()) {
      RCLCPP_WARN(this->get_logger(), "IMU buffer is empty, skipping deskewing");
      // [REVIEW FIX 2026-07-08 P2] Sensor-frame cloud (scan_in_world_frame_
      // stays false); refresh T_prior (previously left stale here).
      this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
      this->current_scan = this->original_scan;
      this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
      return true;
    }

    // Check if oldest IMU is before prev_scan_stamp (need some margin)
    double oldest_imu_time = this->imu_buffer.back().stamp;
    double margin = 0.1;  // 100ms margin

    if (oldest_imu_time > this->prev_scan_stamp - margin) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for sufficient IMU history (oldest: %.3f, need: %.3f). Skipping deskewing.",
                           oldest_imu_time, this->prev_scan_stamp);
      this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
      // As above, a successful registration against this scan is a
      // median-time measurement even though its initial guess was not
      // propagated. Advancing this stamp lets the next frame use the newly
      // accepted pose as its honest IMU integration seed.
      this->t_prior_stamp_ = trajectory.measurement_stamp;
      pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
      this->current_scan = deskewed_scan_;
      this->scan_in_world_frame_ = true;
      this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
      return true;
    }
  }

  // IMU prior & deskewing
  RCLCPP_DEBUG(this->get_logger(),
               "Integrating IMU: prev_stamp=%.3f, pos=[%.2f,%.2f,%.2f], vel=[%.2f,%.2f,%.2f]",
               this->prev_scan_stamp,
               this->basePose.p.x(), this->basePose.p.y(), this->basePose.p.z(),
               this->prev_vel.x(), this->prev_vel.y(), this->prev_vel.z());

  // [P1 FIX 2026-07-10] Integrate from the time basePose is actually VALID at
  // — the previous scan's MEDIAN point time (base_pose_stamp_) — not the
  // previous HEADER stamp. basePose is stored at median time (the accepted
  // candidate / T_prior are median-time poses); starting the integration at
  // the header re-integrated the ~half-sweep of rotation basePose already
  // contains: a systematic yaw LEAD of yaw_rate * (median - header), ~1.5 deg
  // at 30 deg/s with a 50 ms offset — exactly the soft yaw-veto threshold,
  // biasing every turn frame toward the veto. Falls back to prev_scan_stamp
  // when the stamp was never populated (pre-first-scan states).
  const double seed_time =
      (this->base_pose_stamp_ > 0.0) ? this->base_pose_stamp_ : this->prev_scan_stamp;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
  std::vector<Eigen::Vector3f> velocities;
  const auto imu_integrate_start = std::chrono::steady_clock::now();
  frames = this->integrateImu(seed_time, this->basePose.q, this->basePose.p,
                              this->prev_vel, trajectory.times, &velocities);
  this->last_deskew_imu_integrate_ms_ =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - imu_integrate_start)
          .count();

  // If there are no frames between the start and end of the sweep, use previous transform
  if (frames.size() != trajectory.times.size() ||
      velocities.size() != trajectory.times.size()) {
    // [REVIEW FIX 2026-07-08 P3] Snapshot buffer stats under mtx_imu — the
    // IMU callback push_fronts concurrently, so even this log-only read of
    // size()/back() was a data race.
    size_t imu_buf_size = 0;
    double imu_oldest_stamp = 0.0;
    {
      std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
      imu_buf_size = this->imu_buffer.size();
      if (!this->imu_buffer.empty()) imu_oldest_stamp = this->imu_buffer.back().stamp;
    }
    RCLCPP_WARN(this->get_logger(),
                "IMU integration failed! Got %lu frames for %lu timestamps. "
                "Time range: [%.3f, %.3f], IMU buffer size: %lu, first IMU: %.3f",
                frames.size(), trajectory.times.size(),
                this->prev_scan_stamp, trajectory.times.back(),
                imu_buf_size, imu_oldest_stamp);
    this->T_prior = this->basePoseMatrix();  // [REVIEW FIX 2026-07-08] basePose (INS-prior-corrected), not current_pose
    // The motion prior was not advanced, but the GICP measurement produced
    // from this cloud is still valid at the current sweep's median time.  Keep
    // that measurement time separate from the stale seed time.  In particular,
    // failure recovery must query GT/VKS at this scan, not at the last accepted
    // frame (which may already have aged out of the bounded GT buffer).
    this->t_prior_stamp_ = trajectory.measurement_stamp;
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    this->scan_in_world_frame_ = true;
    this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
    return true;
  }

  // A healthy deskew completes at scan rate.  Keep its detailed timing trace
  // for opt-in tuning only; malformed timestamps and deskew failures still
  // report through their normal warnings/errors.
  if (this->verbose_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Deskewing OK: %lu trajectory knots for %lu points, "
                         "scan time [%.3f, %.3f]",
                         frames.size(), point_timestamps.size(),
                         trajectory.times.front(), trajectory.times.back());
  }

  // Update prior to be the estimated pose at the median time of the scan
  this->T_prior = frames[median_knot_index];
  this->T_prior_velocity_ = velocities[median_knot_index];
  // [REVIEW FIX 2026-07-08] ...and record that time: basePose inherits it via
  // the accept/reject paths in performLocalization, and the INS heading prior
  // queries the INS buffer at exactly this stamp on the next scan.
  this->t_prior_stamp_ = trajectory.measurement_stamp;

  // Pre-compose world<-lidar once per trajectory knot. The hot point loop then
  // applies a bounded 2 ms affine segment directly with R*p+t; it does not
  // construct/slerp quaternions or multiply 4x4 matrices per ray.
  const auto transform_start = std::chrono::steady_clock::now();
  const DeskewAffineTrajectory affine_trajectory =
      buildDeskewAffineTrajectory(trajectory.times, frames,
                                  this->extrinsics.baselink2lidar_T);
  if (!affine_trajectory.valid()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Deskew affine trajectory construction failed; dropping scan");
    return false;
  }

  std::atomic<bool> interpolation_failed{false};
  #pragma omp parallel for
  for (size_t i = 0; i < point_timestamps.size(); ++i) {
    auto &pt = deskewed_scan_->points[i];
    Eigen::Vector3f transformed;
    if (!interpolateDeskewPointAffine(
            affine_trajectory, point_timestamps[i],
            Eigen::Vector3f(pt.x, pt.y, pt.z), &transformed)) {
      interpolation_failed.store(true, std::memory_order_relaxed);
      continue;
    }
    pt.x = transformed.x();
    pt.y = transformed.y();
    pt.z = transformed.z();
  }
  if (interpolation_failed.load(std::memory_order_relaxed)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Deskew pose interpolation failed; dropping scan");
    return false;
  }
  this->last_deskew_transform_ms_ =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - transform_start)
          .count();

  this->current_scan = deskewed_scan_;
  this->scan_in_world_frame_ = true;
  this->prev_scan_stamp = this->scan_stamp.seconds();
  return true;
}

// Sensor-frame spatial gate. Applied to the raw lidar-frame cloud BEFORE deskew
// (see the call site in the scan handler). The radial limits mirror
// perception-ws gicp_localizer's scan_min_range/scan_max_range contract; the
// optional axis-aligned crop remains for backwards-compatible DLIO profiles.
// It must NOT run after deskew, where points are in the world frame and an
// origin-centred gate would clip scans far from the map origin.
void gicp_localizer::GicpLocalizer::cropBoxFilterSensorFrame(pcl::PointCloud<PointType>::Ptr& cloud) {
  if (!cloud || cloud->points.empty()) return;
  const bool apply_box = this->crop_size_ > 0.0 && this->crop_size_ < 1000.0;
  const bool apply_range =
      this->scan_min_range_ > 0.0 || this->scan_max_range_ > 0.0;
  if (!apply_box && !apply_range) return;

  const size_t original_size = cloud->points.size();
  const float min_range_sq = static_cast<float>(
      this->scan_min_range_ * this->scan_min_range_);
  const float max_range_sq = static_cast<float>(
      this->scan_max_range_ * this->scan_max_range_);
  pcl::PointCloud<PointType> filtered;
  filtered.header = cloud->header;
  filtered.is_dense = cloud->is_dense;
  filtered.sensor_origin_ = cloud->sensor_origin_;
  filtered.sensor_orientation_ = cloud->sensor_orientation_;
  filtered.points.reserve(original_size);
  for (const auto& point : cloud->points) {
    const float range_sq =
        point.x * point.x + point.y * point.y + point.z * point.z;
    if (!std::isfinite(range_sq) || range_sq < min_range_sq ||
        (this->scan_max_range_ > 0.0 && range_sq > max_range_sq)) {
      continue;
    }
    if (apply_box &&
        (std::abs(point.x) > this->crop_size_ ||
         std::abs(point.y) > this->crop_size_ ||
         std::abs(point.z) > this->crop_size_)) {
      continue;
    }
    filtered.points.push_back(point);
  }
  filtered.width = static_cast<uint32_t>(filtered.points.size());
  filtered.height = 1;
  *cloud = std::move(filtered);
  RCLCPP_DEBUG(
      this->get_logger(),
      "Sensor-frame spatial gate: %lu -> %lu points (range %.1f..%.1f m, box %.1f m)",
      original_size, cloud->points.size(), this->scan_min_range_,
      this->scan_max_range_, this->crop_size_);
}

void gicp_localizer::GicpLocalizer::preprocessPointCloud(pcl::PointCloud<PointType>::Ptr& cloud) {

  size_t original_size = cloud->points.size();
  (void)original_size;

  // NOTE: the crop box is intentionally NOT applied here. After deskew the cloud
  // is in the world frame, so an origin-centered box would clip the scan far from
  // the map origin. Cropping happens in cropBoxFilterSensorFrame() before deskew.

  // Voxel filter
  if (this->vf_use_) {
    size_t before_voxel = cloud->points.size();
    pcl::PointCloud<PointType> filtered;
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);
    voxel.setInputCloud(cloud);
    voxel.filter(filtered);
    if (filtered.points.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Voxel filter removed ALL %lu input points (res=%.3f m) — "
                  "check point coordinates and frame; keeping unfiltered cloud for this scan",
                  before_voxel, this->vf_res_);
      // Leave cloud unchanged — no copy needed.
    } else {
      *cloud = std::move(filtered);
      const double reduction = 1.0 - static_cast<double>(cloud->points.size()) /
                                         static_cast<double>(before_voxel);
      if (reduction > 0.99) {
        RCLCPP_WARN(this->get_logger(),
                    "Voxel filter removed %.1f%% of points (%lu -> %lu, res=%.3f m) — "
                    "check point frame/coordinates",
                    reduction * 100.0, before_voxel, cloud->points.size(), this->vf_res_);
      }
      RCLCPP_DEBUG(this->get_logger(), "Voxel filter: %lu -> %lu points", before_voxel, cloud->points.size());
    }
  }

  // Keep the spatial resolution used by normal frames, but bound the dense-
  // scene tail. This runs after VoxelGrid because its deterministic voxel
  // order lets an even index sample retain broad coverage at O(N) copy cost.
  std::vector<size_t> budget_indices;
  if (this->scan_point_budget_spatial_balancing_) {
    const Eigen::Matrix4f lidar_pose =
        this->T_prior * this->extrinsics.baselink2lidar_T;
    const std::array<double, 3> sensor_origin = {
        static_cast<double>(lidar_pose(0, 3)),
        static_cast<double>(lidar_pose(1, 3)),
        static_cast<double>(lidar_pose(2, 3))};
    budget_indices = spatiallyBalancedPointIndices(
        cloud->points, this->scan_max_points_, sensor_origin,
        this->scan_max_range_, this->scan_point_budget_azimuth_bins_,
        this->scan_point_budget_range_bins_);
  } else {
    budget_indices = evenlySpacedPointIndices(
        cloud->points.size(), this->scan_max_points_);
  }
  if (!budget_indices.empty()) {
    const size_t before_budget = cloud->points.size();
    pcl::PointCloud<PointType> budgeted;
    budgeted.header = cloud->header;
    budgeted.is_dense = cloud->is_dense;
    budgeted.sensor_origin_ = cloud->sensor_origin_;
    budgeted.sensor_orientation_ = cloud->sensor_orientation_;
    budgeted.points.reserve(budget_indices.size());
    for (const size_t index : budget_indices) {
      budgeted.points.push_back(cloud->points[index]);
    }
    budgeted.width = static_cast<uint32_t>(budgeted.points.size());
    budgeted.height = 1;
    *cloud = std::move(budgeted);
    RCLCPP_DEBUG(
        this->get_logger(), "Source point budget (%s): %lu -> %lu points",
        this->scan_point_budget_spatial_balancing_ ? "spatial" : "ordered",
        before_budget, cloud->points.size());
  }

  RCLCPP_DEBUG(this->get_logger(), "Preprocessing: %lu -> %lu points total", original_size, cloud->points.size());
}
