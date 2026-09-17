#include "gicp_interface/detail/localizer_utils.hpp"

using gicp_localizer::detail::auxSchemaMatchesPrimary;
using gicp_localizer::detail::clearPointTimeUnion;
using gicp_localizer::detail::copyPointTimeFromCloud;
using gicp_localizer::detail::findTimeField;
using gicp_localizer::detail::findXYZOffsets;
using gicp_localizer::detail::logLuminarTimestampStats;
using gicp_localizer::detail::logTimestampDiagnostic;
using gicp_localizer::detail::luminarCloudUsesRelativeFloat64;
using gicp_localizer::detail::luminarFloat64TimeContractMatches;
using gicp_localizer::detail::luminarTimestampRangeFromCloud;
using gicp_localizer::detail::luminarUsesRawEpochCarrier;
using gicp_localizer::detail::poseSummary;
using gicp_localizer::detail::shiftCloudTimestamps;
using gicp_localizer::detail::transformCloudData;

void gicp_localizer::GicpLocalizer::callbackPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_in) {

  if (this->imu_only_mode_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "IMU-only mode enabled: skipping pointcloud/GICP updates.");
    return;
  }

  if (this->sync_active_) {
    // Luminar production path: validate + enqueue only. The worker owns
    // release order and runs the pipeline; this callback must never block on
    // either aux state or a long GICP iteration, because blocking can exhaust
    // DDS keep-last history and silently lose front clouds.
    this->enqueuePrimary(pc_in);
    return;
  }

  // Legacy synchronous path (non-Luminar sensor).
  // [P3 FIX 2026-07-14] Catch a pipeline exception (e.g. strict-merge abort)
  // HERE, on whatever executor thread ran this callback. The try/catch around
  // executor.spin() in main() only covers the single spin-calling thread; under
  // a MultiThreadedExecutor this callback can run on any of the other N-1
  // threads, whose uncaught exception bypasses that catch and reaches
  // std::terminate. Mirror the sync worker's controlled-shutdown handling so
  // main() converts it into a nonzero exit code via syncFatal().
  try {
    this->processScan(pc_in);
  } catch (const std::exception& e) {
    this->sync_fatal_.store(true);
    RCLCPP_FATAL(this->get_logger(),
                 "legacy scan pipeline threw: %s — initiating controlled shutdown", e.what());
    rclcpp::shutdown();
  }
}

// Front-cloud admission: the ONLY reasons a front cloud is not enqueued are
// independently reported front failures (null/empty/malformed primary data).
// Aux state and queue pressure can never cause a front drop.
void gicp_localizer::GicpLocalizer::enqueuePrimary(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc) {
  if (!pc || pc->width * pc->height == 0 || pc->data.empty()) {
    std::lock_guard<std::mutex> lk(this->sync_mtx_);
    ++this->front_received_;
    ++this->front_invalid_;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "front sync: rejecting empty/malformed primary cloud "
                         "(front_invalid=%lu)",
                         static_cast<unsigned long>(this->front_invalid_));
    return;
  }

  // Intentional registration-rate admission happens before point-time decode,
  // cloud copies, aux synchronization, deskew and GICP. It is based on the
  // sensor timestamp (not callback wall time), so replay speed cannot change
  // which scans are selected. A 2 Hz deployment consuming a healthy 10 Hz
  // sensor reports rate_limited=80% and overload_dropped=0.
  {
    std::lock_guard<std::mutex> lk(this->sync_mtx_);
    ++this->front_received_;
    const int64_t stamp_ns = rclcpp::Time(pc->header.stamp).nanoseconds();
    if (!this->registration_rate_limiter_.admit(stamp_ns)) {
      ++this->front_rate_limited_;
      return;
    }
  }

  if (!this->concat_float64_contract_checked_.exchange(true)) {
    std::string contract_detail;
    if (!luminarFloat64TimeContractMatches(
            *pc, this->concat_float64_time_is_epoch_ns_, contract_detail)) {
      if (this->concat_float64_time_fail_on_mismatch_) {
        {
          std::lock_guard<std::mutex> lk(this->sync_mtx_);
          ++this->front_invalid_;
          this->sync_fatal_.store(true);
        }
        RCLCPP_FATAL(
            this->get_logger(),
            "Luminar FLOAT64 point-time carrier contradicts "
            "localization/lidar_concat/float64_time_is_epoch_ns=%s: %s",
            this->concat_float64_time_is_epoch_ns_ ? "true" : "false",
            contract_detail.c_str());
        rclcpp::shutdown();
        return;
      }
      RCLCPP_ERROR(
          this->get_logger(),
          "Luminar FLOAT64 point-time carrier mismatch ignored by explicit "
          "escape hatch: %s",
          contract_detail.c_str());
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "Luminar FLOAT64 point-time carrier validated (%s): %s",
          this->concat_float64_time_is_epoch_ns_ ? "raw epoch ns"
                                                : "relative seconds",
          contract_detail.c_str());
    }
  }

  PendingPrimaryCloud pending;
  pending.msg = pc;
  // Decode ONCE. An invalid range (unsupported time field) means point-time
  // matching is impossible: release immediately and let mergeAuxClouds record
  // the per-aux outcome — the front cloud itself is still processed.
  pending.range = luminarTimestampRangeFromCloud(
      *pc, this->concat_float64_time_is_epoch_ns_);
  pending.relative_float64_time = luminarCloudUsesRelativeFloat64(
      *pc, this->concat_float64_time_is_epoch_ns_);
  const auto now = std::chrono::steady_clock::now();
  pending.enqueued = now;
  pending.deadline =
      (pending.range.valid || pending.relative_float64_time)
          ? now + std::chrono::duration<int64_t, std::nano>(
                      static_cast<int64_t>(this->concat_future_aux_wait_s_ * 1e9))
          : now;

  {
    std::lock_guard<std::mutex> lk(this->sync_mtx_);
    // [P3 FIX 2026-07-14] Worker is draining or already joined: account this
    // late front as shutdown_unprocessed instead of queuing it (the worker will
    // never release it, so it would otherwise orphan in primary_queue_ and read
    // as a spurious conservation violation). Keeps received == accounted at all
    // times so the locked summary snapshot in drainFrontSync is consistent.
    if (this->sync_shutdown_.load()) {
      ++this->front_shutdown_unprocessed_;
      return;
    }
    // Compute-overload policy — the queue IS a hard bound. One worker runs
    // one full GICP pipeline per front; when the solver is slower than the
    // input rate an unbounded backlog would grow until queued scans outlive
    // the 2000-sample IMU history and deskew degrades. Coalesce by dropping
    // the OLDEST queued front (its IMU window is the one at risk), loudly and
    // counted: this is the ONLY code path that may drop a front, it is never
    // taken for aux reasons, and it enters the conservation invariant.
    while (this->primary_queue_.size() >= this->concat_primary_queue_size_) {
      this->primary_queue_.pop_front();
      ++this->front_overload_dropped_;
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "front sync: COMPUTE OVERLOAD — GICP pipeline is slower than the "
          "front input rate; dropped the oldest queued front "
          "(front_overload_dropped=%lu, queue bound %zu). Reduce solver cost "
          "(VGICP/decimation) or raise primary_queue_size.",
          static_cast<unsigned long>(this->front_overload_dropped_),
          this->concat_primary_queue_size_);
    }
    pending.arrival_seq = this->sync_seq_++;
    pending.epoch = this->sync_epoch_.load();
    this->primary_queue_.push_back(std::move(pending));
  }
  this->sync_cv_.notify_all();
}

size_t gicp_localizer::GicpLocalizer::dropQueuedFrontsAfterRegistrationTimeout() {
  size_t dropped = 0;
  uint64_t total_timeout_dropped = 0;
  {
    std::lock_guard<std::mutex> lk(this->sync_mtx_);
    dropped = this->primary_queue_.size();
    while (!this->primary_queue_.empty()) {
      this->primary_queue_.pop_front();
      ++this->front_registration_timeout_dropped_;
    }
    total_timeout_dropped = this->front_registration_timeout_dropped_;
  }
  if (dropped > 0) {
    RCLCPP_ERROR(
        this->get_logger(),
        "front sync: registration timed out; discarded %zu queued stale front(s) "
        "instead of processing backlog (timeout_dropped=%lu)",
        dropped, static_cast<unsigned long>(total_timeout_dropped));
  }
  return dropped;
}

// Synchronizer worker: single owner of front release order. For the oldest
// pending front, each aux is independently `matched` (point-range endpoint
// error <= gate; header distance only as tie-break), `final` (watermark — its
// newest valid point time is already past front.min + gate, so an aligned
// sweep can no longer arrive — or it exposes no absolute point time), or
// `pending`. Release on: all matched/final, deadline expiry, queue pressure,
// or shutdown drain. Fronts are always released in arrival order.  Crucially,
// with require_all_aux=true a timeout/final-mismatch is consumed here rather
// than passed to processScan() for a second, racy aux selection.  That makes a
// missing aux frame a bounded synchronizer event instead of a half-entered
// backend transaction.
void gicp_localizer::GicpLocalizer::syncWorkerLoop() {
  std::unique_lock<std::mutex> lk(this->sync_mtx_);
  while (!this->sync_shutdown_.load()) {
    if (this->primary_queue_.empty()) {
      // Predicate wait: shutdown and enqueue both mutate state under
      // sync_mtx_, so this cannot miss either transition.
      this->sync_cv_.wait(lk, [this] {
        return this->sync_shutdown_.load() || !this->primary_queue_.empty();
      });
      continue;
    }

    auto& front = this->primary_queue_.front();
    bool all_matched = true;
    bool ready = true;
    if (front.range.valid) {
      // Lock order: sync_mtx_ -> aux.mtx (aux callbacks never hold both).
      for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
        auto& aux = *this->aux_lidars_[i];
        const double clock_off = (i < this->concat_aux_time_offsets_.size())
                                     ? this->concat_aux_time_offsets_[i] : 0.0;
        bool matched = false;
        bool any_valid_range = false;
        bool buffer_empty = true;
        LuminarTimestampRangeNs newest;
        {
          std::lock_guard<std::mutex> alk(aux.mtx);
          buffer_empty = aux.buffer.empty();
          for (const auto& buffered : aux.buffer) {
            if (!buffered.luminar_range.valid) continue;
            any_valid_range = true;
            const auto shifted = shiftedRange(buffered.luminar_range, clock_off);
            if (endpointDeltaSeconds(front.range, shifted) <=
                this->concat_luminar_point_threshold_) {
              matched = true;
              break;
            }
            if (!newest.valid || shifted.min_ns > newest.min_ns) {
              newest = shifted;
            }
          }
        }
        if (matched) continue;
        // [P3 FIX 2026-07-14] "aux with no absolute point time => final": an aux
        // whose buffered sweeps carry NO decodable absolute Luminar time can
        // only ever be merged header-nearest — it will never point-time match.
        // Treat it as satisfied instead of blocking every release for the full
        // future-aux timeout (the old behavior added ~concat_future_aux_wait_s
        // to every front while still reporting the release as ALL_MATCHED). An
        // EMPTY buffer is genuinely pending — its sweep just hasn't arrived —
        // so only a non-empty all-invalid buffer counts as final here.
        if (!buffer_empty && !any_valid_range) continue;
        all_matched = false;
        // Watermark: aux stream has already advanced past this front's window.
        if (!luminarWatermarkPassed(front.range, newest,
                                    this->concat_luminar_point_threshold_)) {
          ready = false;
          break;
        }
      }
    } else if (front.relative_float64_time) {
      // Laguna's decoder publishes FLOAT64 seconds-since-sweep-start. There
      // is no absolute point range to compare, so mirror GLIM's safe
      // header-fallback watermark: wait until every aux stream has reached
      // this primary header before selecting the nearest header. Releasing
      // immediately would always choose the latest past side sweep.
      const double primary_header =
          rclcpp::Time(front.msg->header.stamp).seconds();
      for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
        auto& aux = *this->aux_lidars_[i];
        const double clock_off =
            (i < this->concat_aux_time_offsets_.size())
                ? this->concat_aux_time_offsets_[i]
                : 0.0;
        double newest_header = -std::numeric_limits<double>::infinity();
        {
          std::lock_guard<std::mutex> alk(aux.mtx);
          for (const auto& buffered : aux.buffer) {
            newest_header = std::max(
                newest_header,
                rclcpp::Time(buffered.msg->header.stamp).seconds() +
                    clock_off);
          }
        }
        if (newest_header < primary_header) {
          all_matched = false;
          ready = false;
          break;
        }
      }
    }

    // [P2 FIX 2026-07-14] Copy the deadline before waiting. wait_until takes
    // its time_point by const reference and libstdc++ re-reads it after wake;
    // the overload path (enqueuePrimary pop_front) can destroy this `front`
    // element while the lock is released inside wait_until, dangling the
    // reference — UB precisely in the overload regime the policy targets.
    const auto deadline = front.deadline;
    const auto now = std::chrono::steady_clock::now();
    const bool timed_out = now >= deadline;
    if (!ready && !timed_out) {
      this->sync_cv_.wait_until(lk, deadline);
      continue;  // re-evaluate: aux arrival, deadline, or shutdown
    }

    // Decide the release reason before popping.
    int reason;
    if (!front.range.valid && !front.relative_float64_time) {
      // [P3 FIX 2026-07-14] Primary had no decodable absolute point time: the
      // aux-matching block above was skipped entirely, so "all_matched" is
      // vacuously true. Report it distinctly instead of as a healthy match.
      reason = RELEASE_PRIMARY_NO_ABSTIME;
    } else if (ready && all_matched) {
      reason = RELEASE_ALL_MATCHED;
    } else if (ready) {
      reason = RELEASE_WATERMARK;
    } else {
      reason = RELEASE_TIMEOUT;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "front sync: %.0f ms deadline expired waiting for aux; releasing "
          "front (stamp=%.6f) with currently matched auxiliaries",
          1e3 * this->concat_future_aux_wait_s_,
          rclcpp::Time(front.msg->header.stamp).seconds());
    }

    PendingPrimaryCloud released = std::move(this->primary_queue_.front());
    this->primary_queue_.pop_front();
    ++this->front_released_;
    ++this->release_reason_counts_[reason];

    // A strict multi-LiDAR scan can only enter the backend after every aux was
    // matched by the synchronizer.  RELEASE_TIMEOUT and RELEASE_WATERMARK have
    // already established that at least one aux cannot be paired with this
    // front; RELEASE_PRIMARY_NO_ABSTIME cannot establish the required point-time
    // pairing at all.  Previously these fronts were unlocked into processScan(),
    // which reselected aux buffers and eventually returned nullptr from
    // mergeAuxClouds().  Besides doing a second selection after the deadline,
    // that left no atomic "skip -> wait for a clean triple" boundary around the
    // consumer.  Consume the invalid front while still under sync_mtx_ instead.
    const bool strict_sync_failure =
        this->concat_require_all_aux_ && reason != RELEASE_ALL_MATCHED;
    if (strict_sync_failure) {
      ++this->front_strict_sync_skipped_;
      this->sync_resync_pending_ = true;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "front sync: strict three-LiDAR resync consumed front before the "
          "scan pipeline (reason=%d, stamp=%.6f, skipped=%lu); waiting for "
          "the next all-matched triple",
          reason, rclcpp::Time(released.msg->header.stamp).seconds(),
          static_cast<unsigned long>(this->front_strict_sync_skipped_));
      // Do not consume the overload continuity marker here.  If an enqueue
      // coalesced fronts while this aux outage was in progress, the first
      // clean triple must still be treated as a discontinuity by the
      // registration safety gates.  (Updating it before this branch used to
      // make that marker disappear on the skipped frame.)
      continue;
    }

    const bool strict_resync_complete = this->sync_resync_pending_;
    if (strict_resync_complete) {
      this->sync_resync_pending_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "front sync: strict three-LiDAR resync complete at stamp=%.6f; "
                  "returning a fully matched triple to the scan pipeline",
                  rclcpp::Time(released.msg->header.stamp).seconds());
    }

    // These markers belong to a real scan-pipeline handoff.  A strict skip
    // has ended queue ownership, but it has not restored estimator continuity.
    this->last_scan_followed_overload_ =
        this->front_overload_dropped_ >
        this->last_processed_front_overload_count_;
    this->last_processed_front_overload_count_ =
        this->front_overload_dropped_;
    this->last_scan_followed_strict_resync_ = strict_resync_complete;

    lk.unlock();
    // mergeAuxClouds re-selects from the aux buffers under the same point-time
    // criteria the readiness test used (buffers hold ~20 s; the sweep cannot
    // have been evicted), then the unchanged deskew/GICP pipeline runs.  The
    // strict timeout/final-mismatch paths above never reach this point.
    // The pipeline can throw (strict require_all_aux abort past budget): an
    // uncaught exception on this dedicated thread would reach std::terminate
    // and bypass all accounting — convert it into a controlled shutdown that
    // main() turns into a nonzero exit code.
    try {
      this->processScan(released.msg, released.epoch);
    } catch (const std::exception& e) {
      this->sync_fatal_.store(true);
      RCLCPP_FATAL(this->get_logger(),
                   "front sync: scan pipeline threw on the worker thread: %s — "
                   "initiating controlled shutdown; remaining queued fronts are "
                   "accounted, not processed",
                   e.what());
      rclcpp::shutdown();  // no-op if the strict-merge handler already called it
    }
    lk.lock();
    if (this->sync_fatal_.load()) {
      break;  // fall through to drain accounting
    }
  }

  // Shutdown drain: release remaining fronts in order with available matches;
  // never wait for missing auxiliaries. If ROS is already down or the
  // pipeline is fatally broken, processing is impossible — account for them
  // instead of dropping silently.
  while (!this->primary_queue_.empty()) {
    if (rclcpp::ok() && !this->sync_fatal_.load()) {
      PendingPrimaryCloud released = std::move(this->primary_queue_.front());
      this->primary_queue_.pop_front();
      ++this->front_released_;
      ++this->release_reason_counts_[RELEASE_SHUTDOWN_DRAIN];
      this->last_scan_followed_overload_ =
          this->front_overload_dropped_ >
          this->last_processed_front_overload_count_;
      this->last_processed_front_overload_count_ =
          this->front_overload_dropped_;
      this->last_scan_followed_strict_resync_ = false;
      lk.unlock();
      try {
        this->processScan(released.msg, released.epoch);
      } catch (const std::exception& e) {
        this->sync_fatal_.store(true);
        RCLCPP_FATAL(this->get_logger(),
                     "front sync: scan pipeline threw during shutdown drain: %s — "
                     "remaining fronts are accounted, not processed",
                     e.what());
      }
      lk.lock();
    } else {
      ++this->front_shutdown_unprocessed_;
      this->primary_queue_.pop_front();
    }
  }
}

void gicp_localizer::GicpLocalizer::processScan(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc_in,
    uint64_t sync_epoch) {
  this->scan_pipeline_start_ = std::chrono::steady_clock::now();

  // The synchronizer worker intentionally unlocks sync_mtx_ before entering
  // the expensive scan pipeline. Serialize that handoff with epoch reset: if a
  // reset wins the gate first, a front that was already popped is identified by
  // its admission generation and dropped instead of re-seeding reset state.
  std::unique_lock<std::mutex> epoch_scan_lock(this->epoch_scan_mtx_);
  const uint64_t no_sync_epoch = std::numeric_limits<uint64_t>::max();
  if (sync_epoch != no_sync_epoch && sync_epoch != this->sync_epoch_.load()) {
    {
      std::lock_guard<std::mutex> sync_lock(this->sync_mtx_);
      ++this->front_epoch_inflight_dropped_;
    }
    RCLCPP_WARN(this->get_logger(),
                "front sync: dropping in-flight old-epoch front (front_epoch=%lu current_epoch=%lu) "
                "after coordinated reset",
                static_cast<unsigned long>(sync_epoch),
                static_cast<unsigned long>(this->sync_epoch_.load()));
    return;
  }

  // [P1 FIX 2026-07-14] Perform a pending coordinated epoch reset at scan-
  // pipeline entry, on the (serialized) scan thread with no estimator locks
  // held. Doing it here — rather than only on the IMU thread — keeps the reset
  // synchronized with the scan pipeline, so a whole scan never runs half across
  // the reset boundary. The IMU-thread entry (callbackImu) remains as the
  // fallback for imu_only_mode (no scans). exchange() ensures it runs once.
  if (this->epoch_reset_pending_.exchange(false)) {
    this->resetEstimatorForEpochChangeLocked("scan-pipeline", this->epoch_reset_regress_s_.load());
    // This scan may have been popped from the old epoch just before the reset
    // flag was set. Fail closed at the boundary; the next admitted front is
    // tagged with the new generation and seeds a clean estimator.
    if (sync_epoch != no_sync_epoch) {
      std::lock_guard<std::mutex> sync_lock(this->sync_mtx_);
      ++this->front_epoch_inflight_dropped_;
    }
    RCLCPP_WARN(this->get_logger(),
                "dropping scan at epoch-reset boundary to prevent cross-epoch deskew/registration");
    return;
  }

  // Multi-LiDAR concatenation: merge point-time-aligned aux scans into the
  // primary cloud before any other processing. Downstream steps (TF cache,
  // manual field extraction, Luminar timestamp read, Y-flip, deskew, GICP) all
  // run on the merged cloud unchanged — primary frame_id, point_step, and
  // field layout are preserved.
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc =
      this->concat_enabled_ ? this->mergeAuxClouds(pc_in) : pc_in;

  // Strict multi-LiDAR merge (require_all_aux) returns nullptr when it skips an
  // incomplete scan so the degraded cloud is never registered. Drop this scan;
  // IMU/geometric propagation continues until a complete merged scan arrives.
  if (!pc) {
    return;
  }

  // Cache base_link -> lidar extrinsic from TF once. With
  // robot_state_publisher providing the URDF TF tree, this is the true
  // lever arm from the vehicle chassis (base_link) to the LiDAR sensor.
  // Deskewing below chains this as `frames[i] * baselink2lidar_T` so the
  // incoming points stay in their native LiDAR frame until then.
  if (!this->extrinsics_cached_) {
    // Resolve base_link -> lidar WITHOUT live TF first (URDF / static), so replay
    // without /tf_static or robot_state_publisher still localizes. Only fall back
    // to a live TF lookup if neither URDF nor a static transform is configured.
    if (this->resolveBaseLidarExtrinsicOffline(pc->header.frame_id)) {
      this->extrinsics_cached_ = true;
    } else {
      try {
        auto tf_bl = this->tf_buffer->lookupTransform(
            this->base_frame, pc->header.frame_id, tf2::TimePointZero);
        Eigen::Quaternionf q_bl(
            tf_bl.transform.rotation.w, tf_bl.transform.rotation.x,
            tf_bl.transform.rotation.y, tf_bl.transform.rotation.z);
        Eigen::Vector3f t_bl(
            tf_bl.transform.translation.x, tf_bl.transform.translation.y,
            tf_bl.transform.translation.z);
        this->extrinsics.baselink2lidar.R = q_bl.toRotationMatrix();
        this->extrinsics.baselink2lidar.t = t_bl;
        this->extrinsics.baselink2lidar_T.setIdentity();
        this->extrinsics.baselink2lidar_T.block<3, 3>(0, 0) = q_bl.toRotationMatrix();
        this->extrinsics.baselink2lidar_T.block<3, 1>(0, 3) = t_bl;
        this->extrinsics_cached_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Cached baselink->lidar extrinsic from TF '%s': t=[%.3f,%.3f,%.3f]",
                    pc->header.frame_id.c_str(), t_bl.x(), t_bl.y(), t_bl.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for baselink->lidar TF ('%s' -> '%s'): %s. "
                             "Set localization/lidar_concat/urdf_path or localization/base_lidar_transform "
                             "for offline replay without /tf_static.",
                             this->base_frame.c_str(), pc->header.frame_id.c_str(), ex.what());
        return;
      }
    }
    // Apply the configured initial pose once the lever arm is known (either path).
    if (this->extrinsics_cached_ && this->pending_initial_pose_) {
      this->applyInitialPoseFromParams();
    }
  }

  if (!this->initialized) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for initialization (odom or initial pose)...");
    return;
  }

  this->scan_stamp = pc->header.stamp;
  this->last_scan_input_frame_ = pc->header.frame_id;

  // Convert to PCL format using manual field extraction for robustness
  pcl::PointCloud<PointType>::Ptr raw_scan = std::make_shared<pcl::PointCloud<PointType>>();

  // Calculate number of points
  size_t num_points = static_cast<size_t>(pc->width) * pc->height;

  RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2: width=%d, height=%d, num_points=%lu, data_size=%lu",
               pc->width, pc->height, num_points, pc->data.size());

  if (num_points == 0) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud (width=%d, height=%d)", pc->width, pc->height);
    return;
  }
  // [P3 FIX 2026-07-10] Non-concat parity with the merge path's tight-cloud
  // check: a truncated message (data.size() < width*height*point_step) would
  // otherwise be read past data.end() by the field-extraction walk below.
  if (pc->point_step == 0 ||
      pc->data.size() < num_points * static_cast<size_t>(pc->point_step)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Dropping truncated PointCloud2 (data=%zu < %zu points x step %u)",
                         pc->data.size(), num_points, pc->point_step);
    return;
  }

  // Single-pass conversion: resolve field offsets once, then walk pc->data
  // exactly once doing xyz + intensity + per-point time + flip_y in the same
  // iteration.
  int x_off = -1, y_off = -1, z_off = -1, i_off = -1;
  uint8_t i_type = 0;
  for (const auto& field : pc->fields) {
    if (field.name == "x") x_off = static_cast<int>(field.offset);
    else if (field.name == "y") y_off = static_cast<int>(field.offset);
    else if (field.name == "z") z_off = static_cast<int>(field.offset);
    else if (field.name == "intensity") {
      i_off = static_cast<int>(field.offset);
      i_type = field.datatype;
    }
  }

  int time_off = -1;
  uint8_t time_datatype = 0;
  int time_count = 0;
  const bool has_time_field = findTimeField(*pc, time_off, time_datatype, time_count);
  this->luminar_scan_time_is_epoch_ns_ =
      this->sensor == gicp_localizer::SensorType::LUMINAR && has_time_field &&
      luminarUsesRawEpochCarrier(
          time_datatype, time_count,
          this->concat_float64_time_is_epoch_ns_);

  // One-shot timestamp-field diagnostic. Fires exactly once across the whole
  // node lifetime (std::call_once) and dumps every PointField + the first few
  // points' timestamp bytes interpreted four ways. The developer reads the
  // [LUMINAR_TS_DIAG] block in stderr to decide which bit-level interpretation
  // the live driver actually uses. See
  // docs/luminar_timestamp_diagnostic_guide.pdf for how to interpret the
  // output and the corresponding fix in copyPointTimeFromCloud.
  static std::once_flag ts_diag_once;
  std::call_once(ts_diag_once, [&]() {
    const char* sensor_name =
        this->sensor == gicp_localizer::SensorType::LUMINAR  ? "luminar"
        : this->sensor == gicp_localizer::SensorType::OUSTER ? "ouster"
        : this->sensor == gicp_localizer::SensorType::VELODYNE ? "velodyne"
        : this->sensor == gicp_localizer::SensorType::HESAI   ? "hesai"
        : this->sensor == gicp_localizer::SensorType::LIVOX   ? "livox"
                                                    : "unknown";
    logTimestampDiagnostic(*pc, time_off, time_datatype, time_count,
                           sensor_name);
  });

  if (x_off < 0 || y_off < 0 || z_off < 0) {
    RCLCPP_ERROR(this->get_logger(), "Point cloud missing x/y/z fields");
    return;
  }

  if (this->deskew_ && !has_time_field) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "enable_deskew enabled but point cloud has no per-point time field "
                         "(t/time/timestamp)");
  } else if (this->sensor != gicp_localizer::SensorType::UNKNOWN && !has_time_field) {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "No per-point time field (t/time/timestamp) in cloud; deskew will not "
                     "work until one is present");
  }

  raw_scan->points.resize(num_points);
  raw_scan->width = pc->width;
  raw_scan->height = pc->height;
  raw_scan->is_dense = pc->is_dense;

  const bool flip_y = this->flip_y_;
  const uint32_t point_step = pc->point_step;
  const uint8_t* base = pc->data.data();

  // Per-point body templated on the intensity reader, so the dispatch happens
  // once outside the loop and the compiler can inline + auto-vectorize.
  auto run = [&](auto&& read_intensity) {
    for (size_t i = 0; i < num_points; ++i) {
      const uint8_t* src = base + i * point_step;
      auto& dst = raw_scan->points[i];

      float x, y, z;
      std::memcpy(&x, src + x_off, sizeof(float));
      std::memcpy(&y, src + y_off, sizeof(float));
      std::memcpy(&z, src + z_off, sizeof(float));
      dst.x = x;
      dst.y = flip_y ? -y : y;
      dst.z = z;
      dst.intensity = read_intensity(src);
      clearPointTimeUnion(dst);
      if (has_time_field) {
        copyPointTimeFromCloud(
            src, time_off, time_datatype, time_count, point_step, this->sensor,
            this->concat_float64_time_is_epoch_ns_, dst);
      }
    }
  };

  try {
    if (i_off < 0) {
      run([](const uint8_t*) { return 0.0f; });
    } else {
      const uint8_t i_type_local = i_type;
      const int i_off_local = i_off;
      switch (i_type_local) {
        case sensor_msgs::msg::PointField::FLOAT32:
          run([i_off_local](const uint8_t* src) {
            float v;
            std::memcpy(&v, src + i_off_local, sizeof(float));
            return v;
          });
          break;
        case sensor_msgs::msg::PointField::UINT16:
          run([i_off_local](const uint8_t* src) {
            uint16_t v;
            std::memcpy(&v, src + i_off_local, sizeof(uint16_t));
            return static_cast<float>(v);
          });
          break;
        case sensor_msgs::msg::PointField::UINT8:
          run([i_off_local](const uint8_t* src) {
            return static_cast<float>(*(src + i_off_local));
          });
          break;
        case sensor_msgs::msg::PointField::FLOAT64:
          run([i_off_local](const uint8_t* src) {
            double v;
            std::memcpy(&v, src + i_off_local, sizeof(double));
            return static_cast<float>(v);
          });
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unknown intensity type %d, ignoring", i_type_local);
          run([](const uint8_t*) { return 0.0f; });
          break;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during point cloud conversion: %s", e.what());
    return;
  }

  this->last_raw_point_count_ = raw_scan->points.size();

  if (this->sensor == gicp_localizer::SensorType::LUMINAR && has_time_field && this->verbose_ &&
      !raw_scan->points.empty()) {
    logLuminarTimestampStats(
        raw_scan->points.size(), *raw_scan, 0,
        this->luminar_scan_time_is_epoch_ns_);
  }

  // Store as original scan for deskewing
  this->original_scan = raw_scan;

  // Save dt for geometric observer BEFORE deskew (which overwrites prev_scan_stamp)
  this->observer_dt_ = (this->prev_scan_stamp > 0.0)
                        ? this->scan_stamp.seconds() - this->prev_scan_stamp
                        : 0.0;

  // Crop box filter in SENSOR frame, BEFORE deskew. deskewPointcloud()
  // transforms points into the world frame, so cropping afterward (in
  // preprocessPointCloud) would clip a box centered on the MAP ORIGIN, deleting
  // the whole scan once the vehicle is more than crop_size_ from the origin. A
  // crop box is inherently a sensor-relative near/far-field filter, so it must
  // run here on the raw lidar-frame cloud. This also covers the deskew-fallback
  // paths (which leave the scan in sensor frame).
  this->cropBoxFilterSensorFrame(this->original_scan);

  // Deskew using IMU
  const auto deskew_start = std::chrono::steady_clock::now();
  if (!this->deskewPointcloud()) {
    this->last_deskew_ms_ =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - deskew_start)
            .count();
    return;
  }
  this->last_deskew_ms_ =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - deskew_start)
          .count();

  RCLCPP_DEBUG(this->get_logger(), "After deskewing: current_scan has %lu points",
               this->current_scan->points.size());

  // Preprocess the deskewed scan
  RCLCPP_DEBUG(this->get_logger(), "Before preprocessing: current_scan has %lu points",
               this->current_scan->points.size());

  const auto preprocess_start = std::chrono::steady_clock::now();
  this->preprocessPointCloud(this->current_scan);
  this->last_preprocess_ms_ =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - preprocess_start)
          .count();

  RCLCPP_DEBUG(this->get_logger(), "After preprocessing: current_scan has %lu points",
               this->current_scan->points.size());
  this->last_preprocessed_point_count_ = this->current_scan->points.size();

  if (this->current_scan->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud empty after preprocessing (original had %lu points)",
                raw_scan->points.size());

    if (this->debug_verbose_scan_log_) {
      RCLCPP_WARN(this->get_logger(),
                  "SCAN DEBUG | stamp=%.3f frame=%s raw=%zu pre=0 status=empty_after_preprocess guess=%s",
                  this->scan_stamp.seconds(), this->last_scan_input_frame_.c_str(),
                  this->last_raw_point_count_, poseSummary(this->current_pose).c_str());
    }
    return;
  }

  if (this->verbose_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "After preprocessing: %lu points",
                         this->current_scan->points.size());
  }

  // Perform localization
  RCLCPP_DEBUG(this->get_logger(), "Calling performLocalization()...");
  const bool gicp_accepted = this->performLocalization();
  RCLCPP_DEBUG(this->get_logger(), "performLocalization() completed");

  // The product topics contain validated GICP measurements only. Rejected
  // scans continue updating the internal IMU prior/recovery state, but do not
  // masquerade as a fresh localization solution.
  if (gicp_accepted) {
    RCLCPP_DEBUG(this->get_logger(), "Calling publishPose()...");
    this->publishPose();
    RCLCPP_DEBUG(this->get_logger(), "publishPose() completed");
  }
}

void gicp_localizer::GicpLocalizer::callbackAuxPointCloud(
    int aux_index, sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (aux_index < 0 || static_cast<size_t>(aux_index) >= this->aux_lidars_.size()) {
    return;
  }
  BufferedAuxCloud buffered;
  buffered.msg = std::move(msg);
  if (this->sensor == gicp_localizer::SensorType::LUMINAR) {
    // Decode once here (Reentrant aux group, cheap ~ms scan) so matching and
    // the synchronizer readiness test never re-read cloud bytes.
    buffered.luminar_range = luminarTimestampRangeFromCloud(
        *buffered.msg, this->concat_float64_time_is_epoch_ns_);
  }
  auto& aux = *this->aux_lidars_[aux_index];
  {
    std::lock_guard<std::mutex> lk(aux.mtx);
    aux.buffer.push_back(std::move(buffered));
    while (aux.buffer.size() > this->concat_buffer_size_) {
      aux.buffer.pop_front();
    }
  }
  // Wake the synchronizer under its mutex: notifying without it can lose the
  // wakeup that lands between the worker's readiness check and its wait.
  // Lock order note: aux.mtx was RELEASED above; sync_mtx_ and aux.mtx are
  // never held together here (the worker nests sync_mtx_ -> aux.mtx).
  if (this->sync_active_) {
    std::lock_guard<std::mutex> wake_lk(this->sync_mtx_);
    this->sync_cv_.notify_all();
  }
}

// Resolve every aux LiDAR's T_primary_aux at startup WITHOUT live TF, so the
// multi-LiDAR merge works in offline replay (no robot_state_publisher / no
// /tf_static). Priority per aux: (1) URDF (the same av24.urdf GLIM reads, single
// source of truth), (2) a static row-major 4x4 from yaml, (3) leave unresolved
// so mergeAuxClouds() falls back to a runtime TF lookup. Any aux left unresolved
// here still works online exactly as before.
void gicp_localizer::GicpLocalizer::resolveAuxExtrinsicsOffline(
    const std::vector<std::vector<double>>& static_transforms) {
  // (1) URDF: parse once, resolve primary_frame <- aux.frame for each aux.
  std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> urdf_transforms;
  bool urdf_ok = false;
  if (!this->concat_urdf_path_.empty() && !this->concat_primary_frame_.empty()) {
    try {
      urdf_transforms = gicp_localizer::parse_urdf_transforms(this->concat_urdf_path_);
      urdf_ok = true;
      RCLCPP_INFO(this->get_logger(), "lidar_concat: loaded URDF '%s' (primary_frame='%s')",
                  this->concat_urdf_path_.c_str(), this->concat_primary_frame_.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "lidar_concat: URDF parse failed (%s); falling back to static/TF", e.what());
    }
  }

  for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
    auto& aux = *this->aux_lidars_[i];

    if (urdf_ok) {
      try {
        const Eigen::Isometry3d T = gicp_localizer::compute_transform(
            urdf_transforms, this->concat_primary_frame_, aux.frame);
        aux.T_primary_aux = T.matrix().cast<float>();
        aux.extrinsic_cached = true;
        aux.extrinsic_source = "urdf";
        const Eigen::Vector3f t = aux.T_primary_aux.block<3, 1>(0, 3);
        RCLCPP_INFO(this->get_logger(), "lidar_concat: resolved T(%s <- %s) from URDF: t=[%.3f, %.3f, %.3f]",
                    this->concat_primary_frame_.c_str(), aux.frame.c_str(), t.x(), t.y(), t.z());
        continue;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "lidar_concat: URDF has no %s <- %s chain (%s); trying static/TF",
                    this->concat_primary_frame_.c_str(), aux.frame.c_str(), e.what());
      }
    }

    // (2) Static row-major 4x4 from yaml.
    if (i < static_transforms.size() && static_transforms[i].size() == 16) {
      Eigen::Matrix4f M;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          M(r, c) = static_cast<float>(static_transforms[i][r * 4 + c]);
      aux.T_primary_aux = M;
      aux.extrinsic_cached = true;
      aux.extrinsic_source = "static";
      RCLCPP_INFO(this->get_logger(), "lidar_concat: resolved T(primary <- %s) from static yaml: t=[%.3f, %.3f, %.3f]",
                  aux.frame.c_str(), M(0, 3), M(1, 3), M(2, 3));
      continue;
    }

    // (3) Unresolved -> runtime TF fallback (existing behavior in mergeAuxClouds).
    aux.extrinsic_source = "tf";
    RCLCPP_WARN(this->get_logger(),
                "lidar_concat: aux '%s' has no URDF/static extrinsic; will rely on live TF '%s' <- '%s' "
                "(requires /tf_static at runtime -- set urdf_path or aux_static_transforms for offline replay)",
                aux.topic.c_str(), this->concat_primary_frame_.c_str(), aux.frame.c_str());
  }
}

// Resolve the base_frame <- lidar_frame lever arm WITHOUT live TF, so full GICP
// localization works in offline replay (no robot_state_publisher / /tf_static).
// Priority: (1) URDF (the same av24.urdf used for aux extrinsics, via
// lidar_concat/urdf_path), (2) a static row-major 4x4 from yaml. Returns false if
// neither is available, leaving the caller to fall back to a live TF lookup.
bool gicp_localizer::GicpLocalizer::resolveBaseLidarExtrinsicOffline(const std::string& lidar_frame) {
  auto apply = [this](const Eigen::Matrix4f& T) {
    this->extrinsics.baselink2lidar.R = T.block<3, 3>(0, 0);
    this->extrinsics.baselink2lidar.t = T.block<3, 1>(0, 3);
    this->extrinsics.baselink2lidar_T = T;
  };

  if (lidar_frame == this->base_frame) {
    apply(Eigen::Matrix4f::Identity());
    RCLCPP_INFO(this->get_logger(),
                "Incoming point cloud is already in base frame '%s'; using identity extrinsic",
                this->base_frame.c_str());
    return true;
  }

  // (1) URDF.
  if (!this->concat_urdf_path_.empty()) {
    try {
      auto urdf = gicp_localizer::parse_urdf_transforms(this->concat_urdf_path_);
      const Eigen::Matrix4f T = gicp_localizer::compute_transform(urdf, this->base_frame, lidar_frame).matrix().cast<float>();
      apply(T);
      RCLCPP_INFO(this->get_logger(),
                  "Resolved baselink->lidar (%s <- %s) from URDF: t=[%.3f, %.3f, %.3f]",
                  this->base_frame.c_str(), lidar_frame.c_str(), T(0, 3), T(1, 3), T(2, 3));
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(),
                  "baselink->lidar URDF resolution failed (%s <- %s): %s; trying static/TF",
                  this->base_frame.c_str(), lidar_frame.c_str(), e.what());
    }
  }

  // (2) Static row-major 4x4 from yaml. A calibrated front-LiDAR matrix must
  // never be applied to iris_interface's already transformed base_link cloud.
  if (this->base_lidar_static_.size() == 16) {
    if (lidar_frame != this->base_lidar_static_frame_) {
      RCLCPP_WARN_ONCE(
          this->get_logger(),
          "Ignoring static base-LiDAR transform bound to frame '%s' for incoming frame '%s'; falling back to live TF",
          this->base_lidar_static_frame_.c_str(), lidar_frame.c_str());
      return false;
    }
    Eigen::Matrix4f T;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        T(r, c) = static_cast<float>(this->base_lidar_static_[r * 4 + c]);
    apply(T);
    RCLCPP_INFO(this->get_logger(),
                "Resolved baselink->lidar from static yaml: t=[%.3f, %.3f, %.3f]", T(0, 3), T(1, 3), T(2, 3));
    return true;
  }

  return false;
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr
gicp_localizer::GicpLocalizer::mergeAuxClouds(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary) {

  this->luminar_primary_min_ts_valid_ = false;

  // P4#3: reset the per-frame concat diagnostics. Any early return below
  // leaves them at "nothing merged", which is exactly what happened.
  this->concat_last_merged_aux_ = 0;
  this->concat_last_aux_dt_.assign(this->aux_lidars_.size(),
                                   std::numeric_limits<double>::quiet_NaN());
  this->concat_last_aux_points_.assign(this->aux_lidars_.size(), 0);

  if (this->aux_lidars_.empty()) return primary;

  // Strict-merge failure handler. Single routing point for every "required merge
  // can't complete" path: incomplete aux merge AND primary-precondition failures
  // (missing XYZ, non-tight/padded primary).
  //   - require_all_aux=false -> returns false: degraded merging is allowed, the
  //     caller localizes on whatever LiDARs merged (front + any available aux).
  //   - require_all_aux=true  -> returns true: the caller returns nullptr so the
  //     degraded cloud is NEVER registered (the scan is skipped; IMU propagation
  //     continues). Past max_consecutive_aux_merge_failures the node either aborts
  //     (abort_on_merge_failure=true) or keeps skipping with a louder warning
  //     (abort_on_merge_failure=false). A fully merged scan resets the counter.
  auto on_required_failure = [this](size_t got, const char* reason) -> bool {
    if (!this->concat_require_all_aux_) return false;  // degraded merging allowed
    ++this->concat_consec_fail_;
    const bool over_budget = this->concat_consec_fail_ > this->concat_max_consec_fail_;
    if (over_budget && this->concat_abort_on_merge_failure_) {
      RCLCPP_FATAL(this->get_logger(),
                   "lidar_concat: multi-LiDAR merge REQUIRED but could not complete (%s) for %d consecutive "
                   "scans; abort_on_merge_failure=true -> shutting down. Set require_all_aux=false to localize "
                   "on available LiDARs, or abort_on_merge_failure=false to keep skipping non-fatally.",
                   reason, this->concat_consec_fail_);
      rclcpp::shutdown();
      throw std::runtime_error("lidar_concat: required multi-LiDAR merge failed");
    }
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "lidar_concat: REQUIRED merge incomplete (%zu/%zu aux): %s for %d consecutive scan(s) "
                          "(budget %d)%s -- skipping scan (degraded cloud NOT registered).",
                          got, this->aux_lidars_.size(), reason, this->concat_consec_fail_,
                          this->concat_max_consec_fail_, over_budget ? ", budget exceeded (non-fatal)" : "");
    return true;  // skip this scan
  };

  // Capture the PRIMARY scan's earliest per-point timestamp BEFORE appending any
  // aux cloud. deskewPointcloud() anchors Luminar merged-cloud timing on this --
  // NOT on the global merged minimum. An aux scan that began before the primary
  // carries smaller absolute epoch timestamps; anchoring at the global min would
  // map that aux point to the primary header stamp and deskew the entire merged
  // sweep late (a real motion-prior/deskew bias at AV speeds).
  LuminarTimestampRangeNs primary_luminar_range;
  if (this->sensor == gicp_localizer::SensorType::LUMINAR) {
    // Decode via the shared helper so the anchor matches the per-point reader
    // (copyPointTimeFromCloud) on the accepted absolute encodings (UINT8[8] /
    // FLOAT64); it internally guards short/truncated buffers, and this capture
    // runs BEFORE the tight-cloud guard further down.
    primary_luminar_range = luminarTimestampRangeFromCloud(
        *primary, this->concat_float64_time_is_epoch_ns_);
    if (primary_luminar_range.valid) {
      this->luminar_primary_min_ts_ns_ = primary_luminar_range.min_ns;
      this->luminar_primary_min_ts_valid_ = true;
    }
  }

  const double t_primary = rclcpp::Time(primary->header.stamp).seconds();
  const uint32_t point_step = primary->point_step;
  const std::string& primary_frame = primary->header.frame_id;

  int x_off, y_off, z_off;
  if (!findXYZOffsets(*primary, x_off, y_off, z_off)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "lidar_concat: cannot find xyz fields in primary cloud (frame='%s')",
                         primary_frame.c_str());
    if (on_required_failure(0, "primary cloud missing xyz fields")) return nullptr;
    return primary;
  }

  // Concatenation treats each cloud as a TIGHT array of point_step-sized points
  // (it byte-appends aux data and re-counts by point_step). A row-padded cloud
  // (row_step > width*point_step, i.e. data.size() != width*height*point_step)
  // would make the byte-count include padding. Organized-but-tight (height>1, no
  // padding) is fine to flatten; only padding is rejected. Reject the primary
  // loudly rather than silently miscounting -- Luminar clouds are unorganized and
  // tight (PCAP reader emits height=1, row_step=point_step*width).
  if (point_step == 0 || primary->data.size() != static_cast<size_t>(primary->width) * primary->height * point_step) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "lidar_concat: primary cloud is organized/padded (data=%zu, width=%u, height=%u, step=%u); "
                         "skipping concat (only tight clouds can be byte-appended)",
                         primary->data.size(), primary->width, primary->height, point_step);
    if (on_required_failure(0, "primary cloud organized/padded (non-tight)")) return nullptr;
    return primary;
  }

  // Start the merged cloud as a copy of the primary; we'll append aux bytes.
  auto merged = std::make_shared<sensor_msgs::msg::PointCloud2>(*primary);
  // Count points from the byte buffer (equals width*height for the tight cloud
  // validated above) so merged width/row_step always match the appended bytes.
  size_t total_points = primary->data.size() / point_step;
  size_t merged_aux_count = 0;

  // Reserve once for primary + all aux clouds (assuming roughly equal sizes).
  // Avoids per-aux reallocations as we grow merged->data.
  merged->data.reserve(primary->data.size() * (1 + this->aux_lidars_.size()));

  for (size_t aux_i = 0; aux_i < this->aux_lidars_.size(); ++aux_i) {
    auto& aux = *this->aux_lidars_[aux_i];

    // Luminar safety rule: an invalid primary point-time range (unsupported or
    // malformed time encoding) makes point-coherent matching impossible, and
    // header-nearest selection is NOT a safe substitute — it is exactly the
    // wrong-sweep / 149 ms-span failure mode. Release the front alone and
    // explicitly omit every aux; the header fallback below exists only for
    // non-Luminar sensors.
    const bool relative_float64_time = luminarCloudUsesRelativeFloat64(
        *primary, this->concat_float64_time_is_epoch_ns_);
    if (this->sensor == gicp_localizer::SensorType::LUMINAR &&
        !primary_luminar_range.valid && !relative_float64_time) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "lidar_concat: primary cloud has neither a usable absolute point-time "
          "range nor the supported relative FLOAT64 time contract "
          "(unsupported_point_time); omitting '%s' and merging front-only",
          aux.topic.c_str());
      continue;
    }

    // Cache T_primary_aux from TF on first use. Skip this aux until TF is available.
    if (!aux.extrinsic_cached) {
      try {
        auto tf = this->tf_buffer->lookupTransform(
            primary_frame, aux.frame, tf2::TimePointZero);
        Eigen::Quaternionf q(
            tf.transform.rotation.w, tf.transform.rotation.x,
            tf.transform.rotation.y, tf.transform.rotation.z);
        Eigen::Vector3f t(
            tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z);
        aux.T_primary_aux.setIdentity();
        aux.T_primary_aux.block<3, 3>(0, 0) = q.toRotationMatrix();
        aux.T_primary_aux.block<3, 1>(0, 3) = t;
        aux.extrinsic_cached = true;
        RCLCPP_INFO(this->get_logger(),
                    "lidar_concat: cached T(%s <- %s): t=[%.3f, %.3f, %.3f]",
                    primary_frame.c_str(), aux.frame.c_str(), t.x(), t.y(), t.z());
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "lidar_concat: waiting for TF '%s' -> '%s': %s",
                             primary_frame.c_str(), aux.frame.c_str(), ex.what());
        continue;
      }
    }

    // Match Luminar sweeps by absolute per-point time. Header proximity alone
    // is unsafe on AV-24: an old right sweep can have the nearest header while
    // its rays are one acquisition period early (the 149 ms merged-span bug).
    // Generic sensors retain the header-time fallback because they do not
    // expose an absolute per-point time carrier.
    sensor_msgs::msg::PointCloud2::ConstSharedPtr match;
    // Measured residual point-clock correction (NOT header phase) applied
    // before any time comparison.
    const double aux_clock_off = (aux_i < this->concat_aux_time_offsets_.size())
                                     ? this->concat_aux_time_offsets_[aux_i] : 0.0;
    const bool use_luminar_time =
        this->sensor == gicp_localizer::SensorType::LUMINAR &&
        primary_luminar_range.valid;
    double best_dt = std::numeric_limits<double>::max();
    double best_luminar_time_delta = std::numeric_limits<double>::max();
    {
      std::lock_guard<std::mutex> lk(aux.mtx);
      if (use_luminar_time) {
        std::vector<LuminarSweepCandidate> candidates;
        candidates.reserve(aux.buffer.size());
        for (size_t i = 0; i < aux.buffer.size(); ++i) {
          const auto& buffered = aux.buffer[i];
          if (!buffered.luminar_range.valid) {
            continue;
          }
          const double header_abs_dt = std::abs(
              rclcpp::Time(buffered.msg->header.stamp).seconds() +
              aux_clock_off - t_primary);
          candidates.push_back(LuminarSweepCandidate{
              i, shiftedRange(buffered.luminar_range, aux_clock_off),
              header_abs_dt});
        }
        const auto selection =
            selectClosestLuminarSweep(primary_luminar_range, candidates);
        if (selection) {
          match = aux.buffer[selection->index].msg;
          best_luminar_time_delta = selection->range_delta_s;
          best_dt = selection->header_abs_delta_s;
        }
      } else {
        for (const auto& buffered : aux.buffer) {
          const double dt = std::abs(
              rclcpp::Time(buffered.msg->header.stamp).seconds() +
              aux_clock_off - t_primary);
          if (dt < best_dt) {
            best_dt = dt;
            match = buffered.msg;
          }
        }
      }
    }
    if (use_luminar_time &&
        (!match || best_luminar_time_delta >
                       this->concat_luminar_point_threshold_)) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "lidar_concat: no timestamp-aligned Luminar match for '%s' within "
          "%.3fs of the primary point-time range (best_range_delta=%.3fs, "
          "best_header_delta=%.3fs); dropping aux rather than appending the "
          "wrong sweep",
          aux.topic.c_str(), this->concat_luminar_point_threshold_,
          match ? best_luminar_time_delta : -1.0, match ? best_dt : -1.0);
      continue;
    }
    if (!use_luminar_time &&
        (!match || best_dt > this->concat_time_threshold_)) {
      RCLCPP_DEBUG(this->get_logger(),
                   "lidar_concat: no match for '%s' within %.3fs of primary t=%.3f (best_dt=%.3fs)",
                   aux.topic.c_str(), this->concat_time_threshold_, t_primary,
                   match ? best_dt : -1.0);
      continue;
    }
    // Validate the FULL field schema, not just point_step: the merged cloud
    // keeps the primary's `fields`, so an aux scan with the same point_step but
    // different field offsets/datatypes would be silently misread downstream.
    std::string schema_reason;
    if (!auxSchemaMatchesPrimary(*match, *primary, schema_reason)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — PointCloud2 schema mismatch vs primary: %s. "
                           "(Merged cloud uses the primary field layout; appending mismatched aux bytes "
                           "would misread them. Normalize the aux layout upstream to enable concatenation.)",
                           aux.topic.c_str(), schema_reason.c_str());
      continue;
    }

    // Append aux bytes directly into merged->data, then transform xyz + shift
    // timestamps in place over the just-appended region. No intermediate copy.
    // If validation fails after the append, roll back the resize so a malformed
    // aux scan can't leak into the merged cloud in its own (un-transformed) frame.
    // Reject an organized/padded or otherwise non-tight aux: byte-appending it
    // (or counting by point_step) would desync points from the field layout.
    // Requires data.size() == width*height*point_step (subsumes the multiple-of-
    // point_step check). Organized-but-tight is acceptable; only padding fails.
    if ((match->data.size() % point_step) != 0 ||
        match->data.size() != static_cast<size_t>(match->width) * match->height * point_step) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — non-tight cloud (data=%zu, width=%u, height=%u, step=%u)",
                           aux.topic.c_str(), match->data.size(), match->width, match->height, point_step);
      continue;
    }
    const size_t old_size = merged->data.size();
    merged->data.insert(merged->data.end(), match->data.begin(), match->data.end());
    uint8_t* appended = merged->data.data() + old_size;
    const size_t aux_pts = match->data.size() / point_step;

    int ax, ay, az;
    if (!findXYZOffsets(*match, ax, ay, az)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "lidar_concat: skipping '%s' — no x/y/z fields in PointCloud2",
                           aux.topic.c_str());
      merged->data.resize(old_size);
      continue;
    }
    transformCloudData(appended, aux_pts, point_step, ax, ay, az, aux.T_primary_aux);

    int time_off;
    uint8_t time_dt_type;
    int time_count;
    const bool has_time_field = findTimeField(*match, time_off, time_dt_type, time_count);
    const bool luminar_raw_epoch =
        this->sensor == gicp_localizer::SensorType::LUMINAR &&
        luminarUsesRawEpochCarrier(
            time_dt_type, time_count,
            this->concat_float64_time_is_epoch_ns_);
    // [REVIEW FIX 2026-07-08 P3] For Luminar, "a time field exists" is not
    // "the time field is usable": the decoder accepts ONLY the 8-byte
    // absolute carriers (UINT8[8] raw uint64 epoch ns, or the same bits
    // mislabelled FLOAT64). Any other schema (e.g. a UINT32 relative counter)
    // is rejected per point later, silently collapsing deskew to a rigid
    // transform for rays that were merged as if they carried time. Treat an
    // unsupported Luminar schema like a missing time field here so the aux is
    // DROPPED under deskew instead.
    const bool usable_time_field = has_time_field &&
        (this->sensor != gicp_localizer::SensorType::LUMINAR ||
         time_dt_type == sensor_msgs::msg::PointField::FLOAT64 ||
         (time_dt_type == sensor_msgs::msg::PointField::UINT8 && time_count == 8));
    if (usable_time_field) {
      // Generic relative-time carriers are rebased by header dt. Luminar's
      // absolute carrier ignores dt and applies only the measured residual
      // clock offset — header acquisition phase never touches point times.
      const double dt = rclcpp::Time(match->header.stamp).seconds() + aux_clock_off - t_primary;
      shiftCloudTimestamps(appended, aux_pts, point_step, time_off, time_dt_type, time_count, dt, luminar_raw_epoch,
                           aux_clock_off);
    } else if (this->deskew_) {
      // Without per-point timestamps the aux rays would deskew against the
      // primary scan's IMU integration with a stale (aux-header) reference,
      // smearing them. Drop the aux when deskew is enabled and times are
      // absent or (Luminar) unusable.
      if (has_time_field) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "lidar_concat: skipping '%s' — unsupported Luminar time schema "
                             "(datatype=%u count=%d; need UINT8[8] epoch-ns or "
                             "FLOAT64 relative seconds/explicit epoch-ns)",
                             aux.topic.c_str(), static_cast<unsigned>(time_dt_type), time_count);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "lidar_concat: skipping '%s' — deskew enabled but no time field found",
                             aux.topic.c_str());
      }
      merged->data.resize(old_size);
      continue;
    }

    // Accumulate the byte-derived count (matches the bytes actually appended),
    // not width*height, so merged->width/row_step stay consistent with data.
    total_points += aux_pts;
    ++merged_aux_count;

    // Per-frame + running HEADER-phase diagnostics. signed_dt is deliberately
    // NOT a point-clock estimate: PTP-synchronized Iris units retain a stable
    // acquisition phase (66-92 ms right on AV-24) while their absolute
    // per-point clocks agree to <1 ms. Observability only — copying it into
    // aux_time_offsets would shift an aligned range out of the 10 ms gate.
    const double signed_dt = rclcpp::Time(match->header.stamp).seconds() + aux_clock_off - t_primary;
    this->concat_last_aux_dt_[aux_i] = signed_dt;
    this->concat_last_aux_points_[aux_i] = static_cast<int>(aux_pts);
    aux.dt_sum += signed_dt;
    aux.dt_min = std::min(aux.dt_min, signed_dt);
    aux.dt_max = std::max(aux.dt_max, signed_dt);
    if (++aux.dt_count % 512 == 0) {  // ~every 50 s at 10 Hz
      RCLCPP_INFO(this->get_logger(),
                  "lidar_concat: '%s' header phase vs primary over %lu merges: "
                  "mean=%+.1f ms, min=%+.1f ms, max=%+.1f ms — acquisition phase, "
                  "observability only; NOT point-clock evidence, do not copy into "
                  "aux_time_offsets",
                  aux.topic.c_str(), static_cast<unsigned long>(aux.dt_count),
                  1e3 * aux.dt_sum / static_cast<double>(aux.dt_count),
                  1e3 * aux.dt_min, 1e3 * aux.dt_max);
    }
  }

  this->concat_last_merged_aux_ = static_cast<int>(merged_aux_count);

  // The merged cloud is unorganized (height=1); width = total appended points.
  // Compute row_step in size_t so the point_step*total_points multiply cannot
  // overflow before the (message-mandated) uint32 assignment.
  merged->width = static_cast<uint32_t>(total_points);
  merged->height = 1;
  merged->is_dense = false;
  merged->row_step = static_cast<uint32_t>(static_cast<size_t>(point_step) * total_points);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "lidar_concat: merged %zu/%zu aux scans, total %zu points",
                       merged_aux_count, this->aux_lidars_.size(), total_points);

  // Strict guard: a REQUIRED multi-LiDAR merge that stays incomplete must not be
  // silently localized on fewer LiDARs (the run_5 "merged 0/2" failure mode). The
  // primary-precondition bail-outs above route through the same handler. When
  // require_all_aux is set, an incomplete merge SKIPS the scan (returns nullptr);
  // otherwise the degraded cloud is returned and localized. A fully merged scan
  // resets the budget.
  if (merged_aux_count < this->aux_lidars_.size()) {
    if (on_required_failure(merged_aux_count, "incomplete aux merge")) return nullptr;
  } else {
    this->concat_consec_fail_ = 0;
  }

  return merged;
}


// ---- INS heading/pose prior (division of labor, 2026-07-06) -----------------
// /vks/imu drives IMU-rate propagation + deskew; /vks/filtered_odom
// supplies the STABLE HEADING (and optionally position) prior. The
// gyro-integrated prior chain inherits heading drift from the last accepted
// GICP pose; VKS supplies the selected fused heading in the CG frame.
//
// The correction is applied to basePose BEFORE integrateImu/deskew, so the
// deskewed world-frame cloud, T_prior, the initial guess, the 4-DoF fixed
// axes, the soft rotation-prior target, the yaw veto/innovation gates, and
// the delta-form observer ALL inherit the stable heading consistently.
// (Correcting T_prior after deskew would mislabel an already-placed cloud.)
//
// BLENDED and BOUNDED, never snapped: a constant map-vs-ENU yaw misalignment
// would otherwise be forced into every prior. A persistent nonzero ins_dyaw
// (debug topic / SCAN DEBUG field) MEASURES that misalignment — investigate
// it rather than raising the blend. With this correction in place, the hard
// yaw veto and yaw innovation gates are anchored to a drift-free reference.
