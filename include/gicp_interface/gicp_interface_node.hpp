#ifndef GICP_LOCALIZER__GICP_LOCALIZER_NODE_HPP_
#define GICP_LOCALIZER__GICP_LOCALIZER_NODE_HPP_

// Localization point type and registration helpers.
#include "gicp_interface/delayed_gicp_fusion.hpp"
#include "gicp_interface/imu_range.hpp"
#include "gicp_interface/local_map_grid.hpp"
#include "gicp_interface/luminar_sweep_matching.hpp"
#include "gicp_interface/navsatfix.hpp"
#include "gicp_interface/observer_delayed_correction.hpp"
#include "gicp_interface/point_budget.hpp"
#include "gicp_interface/point_types.hpp"
#include "gicp_interface/quality_gate.hpp"
#include "gicp_interface/registration_rate_limiter.hpp"
#include "gicp_interface/rtk_gate.hpp"
#include "gicp_interface/small_gicp_backend.hpp"
#include "gicp_interface/ttl_track_constraint.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <race_msgs/msg/rde_telemetry.hpp>
#include <race_msgs/msg/target_trajectory_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// BOOST
#include <boost/circular_buffer.hpp>
#include <deque>

// STL
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace gicp_localizer {

// PointType is already defined globally by dlio.h

class GicpLocalizer : public rclcpp::Node {

public:
  // IMU measurement structure (needs to be public for function signatures)
  struct ImuMeas {
    double stamp{0.0};
    double dt{0.0};
    Eigen::Vector3f ang_vel{Eigen::Vector3f::Zero()};
    Eigen::Vector3f lin_accel{Eigen::Vector3f::Zero()};
    // Optional fused absolute attitude carried by sensor_msgs/Imu. The
    // propagation and deskew paths still use only ang_vel/lin_accel; this
    // field is consumed solely by the opt-in heading prior at a time-matched
    // scan seed. orientation_world_base already includes the configured
    // IMU->base rotation.
    bool orientation_valid{false};
    Eigen::Quaternionf orientation_world_base{Eigen::Quaternionf::Identity()};
    double orientation_yaw_variance{-1.0};
  };

  // Ground-truth odom sample (public so internal helper signatures can
  // reference it). p/q are header.frame_id=map; v_lin_body / v_ang_body are in
  // child_frame_id (gt_body). cov_pos_{xx,yy,zz} are the diagonal
  // position-variance terms from pose.covariance[0,7,14] -- carried per-sample
  // so consumers can decide whether the sample is RTK-FIXED quality (init /
  // calibration / heading prior) or merely Atlas's INS-dead-reckoning quality
  // (snap-recovery accepts either).
  struct GtSample {
    double stamp;
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
    Eigen::Vector3f v_lin_body;
    Eigen::Vector3f v_ang_body;
    // Default to +inf so any sample that reaches the RTK gate without having
    // its covariance explicitly populated is treated as NOT RTK-FIXED (the
    // safe direction) rather than reading an uninitialized value. Real
    // samples overwrite these in callbackGtOdom; interpolated samples in
    // getGtPoseAt() carry the conservative max of the bracketing samples.
    double cov_pos_xx = std::numeric_limits<double>::infinity();
    double cov_pos_yy = std::numeric_limits<double>::infinity();
    double cov_pos_zz = std::numeric_limits<double>::infinity();
    // [REVIEW FIX 2026-07-08] Yaw variance (rad^2) from pose.covariance[35],
    // populated by the adapter from Atlas rpy covariance. Default -1 =
    // unpopulated: the INS yaw-quality gate treats <=0 as "no information"
    // and PASSES it (mirrors GLIM gnss_global's
    // orientation_prior_max_yaw_sigma_deg semantics, keeping compat with GT
    // sources that don't fill covariance[35]). Note the deliberate asymmetry vs
    // cov_pos_* (+inf default = fail-closed): position RTK gating has always
    // been mandatory, while yaw quality is an additional opt-out gate on top of
    // it.
    double cov_yaw = -1.0;
  };

  explicit GicpLocalizer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GicpLocalizer();

  void start();
  // Zero preserves rclcpp's hardware-concurrency default. Deployment profiles
  // with a CPU budget may request an explicit executor size before main()
  // begins spinning the node.
  int executorThreadCount() const { return executor_threads_; }

private:
  void getParams();
  bool loadMap();

  void
  callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc);
  void callbackInitialPose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr
          &pose);
  void callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu);
  void callbackGtOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  // Returns true if a GT sample within gt_odom_max_dt_ of `stamp` was found and
  // interpolated into out.
  bool getGtPoseAt(double stamp, GtSample &out);
  // P2#2: world-frame (map) velocity of the gt_body origin by central finite
  // difference of the GT poses bracketing `stamp`. Used by the snap helper
  // when the GT odom's linear twist is unpopulated; returns ~0 at standstill,
  // so it covers both the missing-twist and truly-stationary cases. False when
  // fewer than 2 samples bracket the stamp within gt_odom_max_dt_.
  bool getGtFiniteDiffVelWorld(double stamp, Eigen::Vector3f &v_world_out);
  // Compose T_map_base = T_map_gtbody * inv(T_base_gtbody) using the cached
  // gt_body -> base extrinsic, bringing the GT sample's pose from
  // msg.child_frame_id into base_frame coordinates. The snap helper and the
  // first-message odom-init path use this to ensure they operate in the same
  // body reference as state.p / current_pose. Returns false (and
  // leaves p_out / q_out unmodified) if the extrinsic has not been cached
  // yet; for the AV-24 single-source NA config, gt_body_frame == base_frame
  // so the extrinsic is cached as identity on the first GT message and the
  // composition is a no-op (gt_p_in_base == gt.p, gt_q_in_base == gt.q).
  bool composeGtPoseInBase(const GtSample &gt, Eigen::Vector3f &p_out,
                           Eigen::Quaternionf &q_out) const;
  // Compose GT twist from gt_body into base_frame using the cached
  // T_base_gtbody_ extrinsic. Returns false when gt extrinsics are unavailable.
  bool composeGtTwistInBase(const GtSample &gt, Eigen::Vector3f &v_lin_body_out,
                            Eigen::Vector3f &v_ang_body_out) const;
  // GT-driven pose recovery. Returns true when the snap fired (guards passed
  // and a time-matched GT sample with finite extrinsic was applied to the
  // state).
  // `immediate_after_sync_loss` is intentionally narrower than a normal
  // recovery: it is set only for the first complete triple after the strict
  // synchronizer consumed one or more incomplete fronts.  If that first
  // registration is rejected, waiting for two more bad priors unnecessarily
  // expands the dead-reckoning error before the configured GT recovery can
  // re-anchor it.
  bool maybeSnapPoseToGT(const char *reason, bool force_absolute,
                         bool immediate_after_sync_loss = false);
  // [P3 FIX 2026-07-14] Optional world-frame linear velocity seed. When null
  // (RViz /initialpose, param pose) velocity is zeroed as before; the GT
  // odom-init path passes the message's own twist so a mid-run seed does not
  // start dead-reckoning from v=0.
  void applyInitialPose(const Eigen::Vector3f &p, const Eigen::Quaternionf &q,
                        const rclcpp::Time &stamp, const std::string &source,
                        const Eigen::Vector3f *v_world_lin = nullptr);
  // RTK-driven calibration: accumulate one residual sample if a time-matched GT
  // exists at `stamp`. Returns true if the calibration window has filled and
  // biases were applied (caller should mark imu_calibrated_).
  bool tryRtkCalibrationStep(double stamp, const Eigen::Vector3f &measured_gyro,
                             const Eigen::Vector3f &measured_accel);

  // Is the GT sample RTK-FIXED quality? Tests Atlas-reported pose covariance
  // against rtk_gate_max_pose_var_xy_ / rtk_gate_max_pose_var_z_. Used by
  // consumers (init/calibration/heading prior) that need cm-level truth.
  // maybeSnapPoseToGT does NOT call this -- it accepts any sample because
  // Atlas's INS dead-reckoning is the next-best fallback to GICP failure.
  bool gtSampleIsRtkFixed(const GtSample &s) const;

  void preprocessPointCloud(pcl::PointCloud<PointType>::Ptr &cloud);
  // Sensor-frame crop box; must run BEFORE deskew (world-frame transform).
  void cropBoxFilterSensorFrame(pcl::PointCloud<PointType>::Ptr &cloud);
  // Returns false when the scan cannot be deskewed deterministically (for
  // example, its point-time range is still ahead of the newest IMU sample
  // after the bounded future-IMU wait). The caller must drop that scan rather
  // than clamp its timestamps to callback-dependent IMU availability.
  bool deskewPointcloud();
  // Correct basePose heading (and optionally position) toward the
  // time-matched, RTK-gated INS sample BEFORE IMU integration/deskew.
  void applyInsHeadingPriorToBasePose();
  // Correct only basePose yaw toward the time-matched absolute orientation in
  // the subscribed IMU message. This never reads or applies external position.
  void applyImuHeadingPriorToBasePose();
  bool getImuHeadingAt(double stamp, Eigen::Quaternionf &orientation_world_base,
                       double &yaw_variance, double &time_error_s);
  void applyWorldYawStepToSeedAndObserver(double step_deg);
  bool performLocalization();
  void initializeLocalMapTarget();
  void updateLocalMapTarget(const Eigen::Matrix4f &guess_pose_map,
                            const Eigen::Vector3f &velocity_world);
  void stopLocalMapBuilder();
  void publishPose();
  void publishGicpOdom(const nav_msgs::msg::Odometry &odom_msg);
  void publishNavSatFix(const nav_msgs::msg::Odometry &odom_msg);
  void applyInitialPoseFromParams();

  // [P1 FIX 2026-07-14] Coordinated epoch reset. A large backward stamp jump
  // (bag loop / adapter re-anchor / device power-cycle) invalidates the WHOLE
  // estimator's timestamped state at once; clearing one buffer in isolation
  // leaves the rest cross-epoch-inconsistent and silently corrupts output.
  // resetEstimatorForEpochChangeLocked re-initializes every runtime-resettable
  // piece coherently — imu/gt buffers, timestamp seeds, the init/calibration
  // state machine, and the observer state — so the node re-seeds cleanly on the
  // new epoch (re-applying the param initial pose when configured). It runs on
  // the IMU thread with NO estimator locks held and acquires them in the
  // canonical order pose -> seed -> calib -> gt_odom -> geo -> imu (a
  // consistent superset of every nested acquisition elsewhere), so a concurrent
  // scan/gt/imu callback only ever observes the fully-reset state, never a
  // half-reset one. The scan path then re-converges within a frame or two as
  // prev_scan_stamp (reset to 0) tracks the new epoch. Detection (imuCallback /
  // callbackGtOdom) only sets epoch_reset_pending_; the next scan or IMU
  // callback performs the reset. Caller must hold epoch_scan_mtx_. Keeping the
  // reset and a whole scan under one gate prevents either from observing the
  // other half-complete.
  void resetEstimatorForEpochChangeLocked(const char *source, double regress_s);
  std::atomic<bool> epoch_reset_pending_{false};
  std::atomic<double> epoch_reset_regress_s_{0.0};
  std::mutex epoch_scan_mtx_;
  // Multi-LiDAR concatenation. Aux callbacks decode the absolute point-time
  // range once and buffer it; on the Luminar production path the front
  // callback only validates and enqueues (it never blocks and is never
  // dropped for aux reasons), and the synchronizer worker owns release order:
  // a front cloud is released to the unchanged merge/deskew/GICP pipeline when
  // every aux is matched (point-range endpoint error <= gate, header only as
  // tie-break) or final (watermark: newest aux point time already past the
  // gate), or when its arrival-time deadline expires (merge whatever matched).
  void callbackAuxPointCloud(int aux_index,
                             sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void enqueuePrimary(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc);
  void syncWorkerLoop();
  size_t dropQueuedFrontsAfterRegistrationTimeout();

public:
  // Teardown hook: stops the synchronizer worker, which drains queued fronts
  // in order WHILE the ROS context is still valid. Must be invoked from a
  // pre-shutdown callback (see localization_node.cc) so the run tail is
  // processed rather than counted as shutdown_unprocessed; also called by the
  // destructor as a fallback. Idempotent, thread-safe, and a no-op when
  // called from the worker thread itself (fatal-shutdown path).
  void drainFrontSync();
  bool syncFatal() const { return sync_fatal_.load(); }

private:
  // The pre-existing scan pipeline (merge -> deskew -> GICP); runs on the
  // executor scan thread in legacy mode, on the sync worker otherwise.
  // sync_epoch is captured at front admission. Legacy callers use the sentinel
  // because they do not pass through the asynchronous front queue.
  void processScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_in,
                   uint64_t sync_epoch = std::numeric_limits<uint64_t>::max());
  sensor_msgs::msg::PointCloud2::ConstSharedPtr
  mergeAuxClouds(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &primary);

  // Geometric Observer functions
  void propagateState(const ImuMeas &imu_local);
  void updateState();

  // Timestamp-consistent state path around small_gicp.  IMU/wheel samples are
  // consumed at native rate; an accepted scan pose is applied at its median
  // acquisition time and replayed to the newest IMU before product output.
  void updateDelayedFusionWithImu(const gicp_localizer::detail::ImuSample &sample);
  void updateDelayedFusionWithWheel(double stamp_s,
                                    const Eigen::Vector2d &velocity_body_mps,
                                    const Eigen::Vector2d &variance_mps2);
  bool
  applyDelayedFusionToAcceptedPose(const Eigen::Matrix4f &raw_measurement_pose,
                                   bool wheel_stationary_hold,
                                   DelayedGicpFusionResult *result);

  // IMU integration functions
  // [REVIEW FIX 2026-07-08 P1] Returns a COPY of the needed IMU slice
  // (forward time order) taken while holding mtx_imu. The previous interface
  // handed out boost::circular_buffer iterators that integrateImu()
  // dereferenced lock-free while the (concurrent) IMU callback push_fronts —
  // circular_buffer mutation invalidates/rotates those iterators: normal-path
  // UB that could corrupt T_prior, per-point deskew and the yaw-vs-IMU gates
  // exactly during high-rate turn segments.
  bool imuMeasFromTimeRange(double start_time, double end_time,
                            std::vector<ImuMeas> &out);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImu(double start_time, Eigen::Quaternionf q_init,
               Eigen::Vector3f p_init, Eigen::Vector3f v_init,
               const std::vector<double> &sorted_timestamps,
               std::vector<Eigen::Vector3f> *velocities_out = nullptr);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
                       Eigen::Vector3f v_init,
                       const std::vector<double> &sorted_timestamps,
                       const std::vector<ImuMeas> &imu_slice,
                       std::vector<Eigen::Vector3f> *velocities_out = nullptr);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::
      SharedPtr wheel_speed_sub_;
  rclcpp::Subscription<race_msgs::msg::RdeTelemetry>::SharedPtr
      track_timing_sub_;
  rclcpp::Subscription<race_msgs::msg::TargetTrajectoryCommand>::SharedPtr
      track_trajectory_command_sub_;
  rclcpp::CallbackGroup::SharedPtr pointcloud_cb_group, initial_pose_cb_group,
      imu_cb_group, gt_odom_cb_group, wheel_speed_cb_group_,
      track_timing_cb_group_, track_command_cb_group_;

  // External odom for initialization and failure recovery (optional).
  // Latest message and a small ring buffer for time-matched lookup.
  bool gt_odom_enabled_;
  size_t gt_odom_buffer_size_;
  double gt_odom_max_dt_; // seconds; reject lookups farther than this from scan
                          // stamp
  double gt_interp_max_gap_ =
      0.5; // [P2 FIX 2026-07-14] max bracket width for GT interpolation
  std::deque<GtSample> gt_odom_buffer_;
  std::mutex gt_odom_mtx_;
  std::atomic<bool> gt_odom_received_{false};
  std::string gt_expected_frame_id_;
  std::string gt_expected_child_frame_id_;
  // Constant map-frame translation applied to the external INS/reference
  // position. This follows perception-ws `ins_offset` semantics and makes a
  // map-datum mismatch explicit in configuration instead of hiding a track
  // height correction in the node.
  Eigen::Vector3f ins_offset_{Eigen::Vector3f::Zero()};
  std::atomic<uint64_t> gt_dropped_invalid_{0};
  std::atomic<uint64_t> gt_dropped_frame_{0};

  // RTK quality gate (P1-native), applied PER CONSUMER — not a buffer
  // filter. Every gt_odom sample is buffered; gtSampleIsRtkFixed (finite,
  // nonnegative covariance within pose.covariance[0,7,14] thresholds) gates
  // only RTK bias calibration and the optional INS heading prior. Snap
  // recovery and use_odom_init intentionally accept
  // degraded samples. The gate inspects the gt_odom message itself; no
  // separate status topic is involved. Replaces the old BESTGNSSPOS-enum
  // gate (removed when the NovAtel path was retired).
  bool rtk_gate_enabled_;
  bool rtk_gate_allow_zero_covariance_;
  double rtk_gate_max_pose_var_xy_; // m^2; reject if cov[0] or cov[7] > this
  double rtk_gate_max_pose_var_z_;  // m^2; reject if cov[14] > this
  // Counter for rate-limited rejection logging.
  std::atomic<uint64_t> rtk_rejected_covariance_{0};

  // GT-driven pose recovery (optional). Mirrors the IMU extrinsic caching
  // pattern in callbackImu: on first GT message we record child_frame_id and
  // look up the base_frame ← gt_body TF once. Snap composes T_map_base =
  // T_map_gtbody * inv(T_base_gtbody).
  bool gt_recovery_enabled_;
  int gt_recovery_min_consecutive_failures_;
  int consecutive_failures_; // resets to 0 on accept; increments on any
                             // non-accept
  // True only when the immediately preceding scan passed every generic GICP
  // gate but failed the TTL tangent correction bound. It arms one bounded
  // LiDAR reacquisition attempt; unrelated failures never relax the tangent
  // guard.
  bool previous_failure_track_along_ = false;
  // Recovery runs once per configured-size rejection block. A snap resets the
  // counter, so healthy GICP never consults VKS while a still-lost matcher can
  // recover again after three new rejects instead of dead-reckoning forever.
  // [P2 FIX 2026-07-09] atomic + written LAST inside gt_init_mtx_: the
  // scan/IMU threads read this flag lock-free and must never observe it true
  // before T_base_gtbody_/gt_body_frame_ are fully written.
  std::atomic<bool> gt_extrinsics_cached_;
  Eigen::Matrix4f T_base_gtbody_; // pose of gt_body expressed in base_frame
  std::string gt_body_frame_;     // captured from msg->child_frame_id

  // Multi-LiDAR concatenation
  struct BufferedAuxCloud {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    LuminarTimestampRangeNs luminar_range; // decoded ONCE in the aux callback
  };

  struct AuxLidar {
    std::string topic;
    std::string frame; // header.frame_id of the aux sensor (URDF link)
    Eigen::Matrix4f T_primary_aux; // p_primary = T * p_aux
    bool extrinsic_cached;         // true once T_primary_aux is resolved
    std::string extrinsic_source =
        "tf"; // "urdf" | "static" | "tf" (for logging)
    std::deque<BufferedAuxCloud> buffer;
    std::mutex mtx;
    // Signed header phase vs the primary (aux - primary), accumulated over
    // merged scans. This is useful acquisition-phase evidence but is not, by
    // itself, a PTP/point-clock offset measurement — do not copy it into
    // aux_time_offsets.
    double dt_sum = 0.0;
    double dt_min = std::numeric_limits<double>::infinity();
    double dt_max = -std::numeric_limits<double>::infinity();
    uint64_t dt_count = 0;
  };
  std::vector<std::unique_ptr<AuxLidar>> aux_lidars_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      aux_subs_;
  rclcpp::CallbackGroup::SharedPtr aux_cb_group_;
  bool concat_enabled_;
  double concat_time_threshold_;
  // Luminar acceptance gate: absolute point-time endpoint-range error
  // (max(|min-min|, |max-max|)). Header time is only a tie-break. The 0.1 s
  // header threshold above remains solely for non-Luminar fallback matching.
  double concat_luminar_point_threshold_ = 0.010;
  // Arrival-time (steady-clock) release deadline for a pending front cloud.
  // Protects live latency when an aux packet is lost or its callback stalls;
  // it is NOT a point-clock correction and never alters timestamps.
  double concat_future_aux_wait_s_ = 0.150;
  // HARD bound on the pending-front queue (the compute-overload policy).
  // A single worker runs one full GICP pipeline per front: if the solver is
  // slower than the input rate, an unbounded queue would grow until queued
  // scans outlive the 2000-sample IMU history and deskew degrades. On
  // overflow the OLDEST queued front is dropped with loud, counted
  // accounting (front_overload_dropped_) — the only place a front may be
  // dropped, and never for aux reasons.
  size_t concat_primary_queue_size_ = 8;
  // Optional scan-to-map update-rate cap. Every primary cloud is accounted,
  // but only sensor-stamp-selected scans enter the queue; IMU propagation
  // remains full rate. Protected by sync_mtx_.
  double registration_rate_hz_ = 0.0;
  RegistrationRateLimiter registration_rate_limiter_;
  size_t concat_buffer_size_;
  // Luminar FLOAT64 time fields are scan-relative seconds by default (the
  // Laguna decoder contract). Some drivers mislabel raw uint64 epoch-ns bits
  // as FLOAT64; those require an explicit opt-in so ordinary doubles are
  // never reinterpreted as multi-billion-second timestamps.
  bool concat_float64_time_is_epoch_ns_ = false;
  bool concat_float64_time_fail_on_mismatch_ = true;
  std::atomic<bool> concat_float64_contract_checked_{false};

  // ---- Async Luminar front worker / aux synchronizer ----
  // Contract: every valid front cloud leaves the bounded queue exactly once,
  // in order.  In permissive mode it is released with 0..N_aux auxiliaries.
  // In strict mode, a front that the synchronizer can already prove incomplete
  // is consumed there and never enters the scan pipeline.  This keeps an
  // expired aux wait from becoming a half-processed GICP frame.
  struct PendingPrimaryCloud {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    LuminarTimestampRangeNs range; // decoded ONCE in the front callback
    bool relative_float64_time = false;
    std::chrono::steady_clock::time_point enqueued;
    std::chrono::steady_clock::time_point deadline;
    uint64_t arrival_seq = 0;
    uint64_t epoch = 0; // synchronizer epoch at admission
  };
  enum FrontReleaseReason : int {
    RELEASE_ALL_MATCHED = 0,
    RELEASE_WATERMARK = 1,
    RELEASE_TIMEOUT = 2,
    // 3 (queue_pressure) is reserved and no longer emitted: overload now
    // drops the OLDEST queued front with front_overload_dropped_ accounting
    // instead of releasing it early.
    RELEASE_QUEUE_PRESSURE = 3,
    RELEASE_SHUTDOWN_DRAIN = 4,
    // [P3 FIX 2026-07-14] Primary cloud carried no decodable absolute point
    // time: point-time matching is impossible, so the front is released
    // immediately (header-nearest merge downstream). Formerly counted as
    // RELEASE_ALL_MATCHED, which reported a broken-schema stream as healthy.
    RELEASE_PRIMARY_NO_ABSTIME = 5,
  };
  bool sync_active_ = false; // Luminar: worker owns bounded front processing
  std::deque<PendingPrimaryCloud> primary_queue_; // guarded by sync_mtx_
  std::mutex sync_mtx_;
  std::condition_variable sync_cv_;
  std::thread sync_worker_;
  std::atomic<bool> sync_shutdown_{false};
  uint64_t sync_seq_ = 0;               // guarded by sync_mtx_
  std::atomic<uint64_t> sync_epoch_{0}; // incremented with reset queue purge
  // Conservation counters (sync_mtx_). Invariant, checked in the summary:
  //   front_received_ == front_released_ + front_invalid_ + front_rate_limited_
  //                      + front_shutdown_unprocessed_ +
  //                      front_overload_dropped_
  //                      + front_epoch_dropped_ +
  //                      front_registration_timeout_dropped_
  uint64_t front_received_ = 0;
  uint64_t front_released_ = 0;
  uint64_t front_invalid_ = 0;
  uint64_t front_rate_limited_ =
      0; // intentional admission control, not overload
  uint64_t front_shutdown_unprocessed_ = 0;
  uint64_t front_overload_dropped_ = 0; // compute-overload coalescing drops
  // Worker-owned continuity markers. A front released after one or more
  // admitted scans were coalesced, or the first complete triple after strict
  // auxiliary resynchronization, is a recovery frame even when the previous
  // computed registration happened to be accepted.
  uint64_t last_processed_front_overload_count_ = 0;
  bool last_scan_followed_overload_ = false;
  bool last_scan_followed_strict_resync_ = false;
  uint64_t front_epoch_dropped_ =
      0; // [P2 FIX 2026-07-15] fronts discarded on an epoch reset
  uint64_t front_registration_timeout_dropped_ =
      0; // queued fronts purged after a timed-out solve
  // Strict three-LiDAR synchronizer failures consumed before processScan().
  // They are included in front_released_ (the queue ownership has ended) but
  // kept separately so a replay can distinguish an intentional resync skip
  // from a frame that actually reached deskew/GICP.
  uint64_t front_strict_sync_skipped_ = 0;
  // Guarded by sync_mtx_.  Set by a strict synchronizer-side skip and cleared
  // only when a subsequent front is all-matched.  It is a logical input
  // restart: no estimator state or scan lock is touched while aux is absent.
  bool sync_resync_pending_ = false;
  // Already counted as released when the worker had popped it before reset;
  // kept separate from the conservation invariant for explicit visibility.
  uint64_t front_epoch_inflight_dropped_ = 0;
  uint64_t release_reason_counts_[6] = {0, 0, 0, 0, 0, 0};
  // Set when the scan pipeline throws on the worker thread (e.g. the strict
  // require_all_aux abort): the worker stops processing, requests shutdown,
  // and main() converts this into a nonzero exit code.
  std::atomic<bool> sync_fatal_{false};
  std::mutex drain_mtx_; // makes drainFrontSync() idempotent/thread-safe
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_{};
  double last_deskew_ms_ = 0.0;
  double last_preprocess_ms_ = 0.0;
  double last_deskew_setup_ms_ = 0.0;
  double last_deskew_imu_wait_ms_ = 0.0;
  double last_deskew_imu_integrate_ms_ = 0.0;
  double last_deskew_transform_ms_ = 0.0;
  // Offline aux-extrinsic resolution (no live TF needed). Resolved once at
  // startup: URDF (concat_urdf_path_ + concat_primary_frame_) takes priority,
  // then a static per-aux matrix from yaml, then live TF as a last resort.
  std::string concat_primary_frame_; // URDF link name of the primary LiDAR
  std::string concat_urdf_path_;     // path to av24.urdf ("" = skip URDF)
  // Strict merge guard: when a required multi-LiDAR merge stays incomplete.
  bool concat_require_all_aux_ = false; // false = localize on available LiDARs;
                                        // true = skip incomplete scans
  bool concat_abort_on_merge_failure_ =
      true; // true = abort node past budget; false = keep skipping non-fatally
  int concat_max_consec_fail_ =
      10; // tolerated consecutive incomplete merges (0 = immediate)
  int concat_consec_fail_ =
      0; // running counter of consecutive incomplete merges

  // Per-frame lidar-concat state (scan-callback thread only).
  int concat_last_merged_aux_ = -1; // -1 = concat disabled / not run this frame
  std::vector<double>
      concat_last_aux_dt_; // s, aux header - primary header; NaN = not merged
  std::vector<int> concat_last_aux_points_; // appended points; 0 = not merged
  std::vector<double>
      concat_aux_time_offsets_; // P3 fix: constant per-aux clock offset (s),
                                // order = aux_topics
  double last_scan_time_span_s_ =
      -1.0; // merged-scan per-point time span (deskew path)
  // Resolve every aux's T_primary_aux without live TF; returns the count
  // resolved.
  void resolveAuxExtrinsicsOffline(
      const std::vector<std::vector<double>> &static_transforms);

  // Offline base_frame<-lidar_frame lever arm (no live TF): URDF (lidar_concat/
  // urdf_path) then a frame-bound static yaml matrix. Sets
  // extrinsics.baselink2lidar* and returns true on success; false leaves the
  // caller to fall back to live TF.
  std::vector<double> base_lidar_static_; // row-major 4x4, "" = unset
  std::string base_lidar_static_frame_;   // frame the static matrix describes
  bool resolveBaseLidarExtrinsicOffline(const std::string &lidar_frame);

  // Luminar multi-LiDAR deskew anchor. mergeAuxClouds() captures the PRIMARY
  // scan's earliest per-point timestamp BEFORE appending aux clouds; the deskew
  // LUMINAR branch anchors merged-sweep timing on this instead of the global
  // merged minimum, so an aux scan that began before the primary does not shift
  // the whole sweep late. Reset (valid=false) each scan; only set on the concat
  // path. See deskewPointcloud().
  uint64_t luminar_primary_min_ts_ns_ = 0;
  bool luminar_primary_min_ts_valid_ = false;
  bool luminar_scan_time_is_epoch_ns_ = false;

  // Product publishers.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localized_odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub;

  // TF is consumed for input-frame resolution. Ownership of /tf output stays
  // with vehicle bring-up; this node deliberately publishes no TF.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // Map
  pcl::PointCloud<PointType>::Ptr map_cloud;

  // Current scan
  pcl::PointCloud<PointType>::Ptr current_scan;
  pcl::PointCloud<PointType>::Ptr original_scan;
  rclcpp::Time scan_stamp;
  double prev_scan_stamp;
  std::chrono::steady_clock::time_point scan_pipeline_start_;
  // [REVIEW FIX 2026-07-08] The timestamp basePose actually corresponds to.
  // basePose is set from the accepted candidate / T_prior, which is the pose
  // at the MEDIAN POINT TIME of the scan (frames[median_pt_index]) -- NOT the
  // scan header time stored in prev_scan_stamp. The INS heading prior must
  // query the INS buffer at this stamp; querying at prev_scan_stamp instead
  // produced a yaw-rate-proportional comparison bias on turns
  // (~half-sweep-time * yaw_rate, e.g. 50 ms * 30 deg/s = 1.5 deg).
  double base_pose_stamp_ = 0.0; // time basePose is valid at (0 = unknown)
  double t_prior_stamp_ = 0.0; // time T_prior is valid at for the current scan
  // [REVIEW FIX 2026-07-08 P2] Frame of current_scan as DECLARED by
  // deskewPointcloud(): true = world frame (points placed along the prior
  // chain / at T_prior), false = sensor (lidar) frame. performLocalization()
  // previously inferred the frame from deskew_ alone, but several deskew
  // fallback branches (no IMU yet, unsupported sensor, no per-point
  // timestamps, empty IMU buffer) return the RAW sensor-frame cloud while
  // deskew_ is true — GICP then seeded Identity and composed
  // candidate = solution * T_prior as if the cloud were world-frame,
  // inviting wrong-basin matches at startup / IMU gaps / bad timestamps.
  bool scan_in_world_frame_ = false;
  double observer_dt_;
  std::string last_scan_input_frame_;
  size_t last_raw_point_count_;
  size_t last_preprocessed_point_count_;

  // GICP matcher
  using GicpBackend = SmallGicpBackend<PointType, PointType>;
  using PreparedGicpTarget = GicpBackend::PreparedTargetPtr;
  GicpBackend gicp;

  // Optional local target. The complete prepared map always remains resident
  // and is selected immediately for startup, recovery, or an unsafe crop.
  // Local crops reuse its precomputed point covariances and build only a KD
  // tree on a background thread; the registration thread merely swaps shared
  // pointers at scan boundaries.
  bool local_map_enabled_ = false;
  double local_map_radius_m_ = 200.0;
  double local_map_rebuild_distance_m_ = 40.0;
  double local_map_lead_time_s_ = 0.5;
  double local_map_max_lead_m_ = 25.0;
  double local_map_grid_cell_size_m_ = 50.0;
  size_t local_map_min_points_ = 10000;
  int local_map_builder_threads_ = 1;
  std::unique_ptr<LocalMapGrid<PointType>> local_map_grid_;
  PreparedGicpTarget global_gicp_target_;
  PreparedGicpTarget active_local_gicp_target_;
  Eigen::Vector2f active_local_map_center_ = Eigen::Vector2f::Zero();
  bool active_local_map_center_valid_ = false;
  bool local_map_target_active_ = false;
  size_t local_map_current_target_points_ = 0;
  std::thread local_map_builder_thread_;
  std::atomic<bool> local_map_builder_stop_{false};
  std::atomic<bool> local_map_builder_busy_{false};
  std::mutex local_map_pending_mtx_;
  PreparedGicpTarget pending_local_gicp_target_;
  Eigen::Vector2f pending_local_map_center_ = Eigen::Vector2f::Zero();
  double pending_local_map_build_ms_ = 0.0;
  bool pending_local_map_ready_ = false;
  Eigen::Vector2f requested_local_map_center_ = Eigen::Vector2f::Zero();
  bool requested_local_map_center_valid_ = false;
  double local_map_last_build_ms_ = 0.0;
  uint64_t local_map_build_count_ = 0;
  uint64_t local_map_switch_count_ = 0;
  uint64_t local_map_global_fallback_count_ = 0;
  uint64_t local_map_stale_build_count_ = 0;

  // Current pose estimate
  Eigen::Matrix4f current_pose;
  Eigen::Matrix4f T_prior; // IMU-based prior transformation
  std::atomic<bool> initialized;
  std::mutex pose_mutex;

  bool geo_delta_correction_;

  // Debug tracking
  Eigen::Matrix4f last_gicp_pose_;
  rclcpp::Time last_gicp_stamp_;
  bool last_gicp_valid_;
  double last_fitness_score_{
      -1.0}; // -1 = no scan yet (latest ATTEMPT, incl. rejected)
  // [REVIEW FIX 2026-07-08 P2] Fitness of the last ACCEPTED scan only. The
  // scan-rate odometry covariance keys on this; last_fitness_score_ is written
  // before the accept/reject decision (and can be +inf on a failed applied-
  // pose re-score), so one rejected scan would otherwise publish huge or
  // infinite covariance while last_gicp_valid_ was still true from an older
  // accepted scan.
  double last_accepted_fitness_score_{-1.0};
  double last_accepted_scan_stamp_{
      -1.0}; // s — stamp of last accepted GICP scan (P3 dead-reckon cov)

  // IMU data structures
  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable imu_cv_;
  std::atomic<bool> first_imu_received;
  bool imu_require_topic_allowlist_{false};
  std::vector<std::string> imu_topic_allowlist_;
  bool imu_require_frame_match_{true};
  bool imu_precalibrated_{false};
  // One-shot guard for the defensive IMU header.frame_id consistency check
  // in callbackImu. The vehicle design consumes /vks/imu referenced at CG.
  // If someone re-points imu_topic at a different IMU, the code would
  // silently treat its axes as if they were already expressed at CG.
  // This flag arms a one-time warning so the misconfiguration surfaces at
  // least once in the log.
  std::atomic<bool> imu_frame_id_checked_{false};

  // IMU calibration state
  std::atomic<bool> imu_calibrated_;
  // [REVIEW FIX 2026-07-08 P2] Serializes the init/bias-calibration state
  // machine (init_phase_, RTK/stationary accumulators, finalization). The IMU
  // subscription is in a REENTRANT callback group under a
  // MultiThreadedExecutor, so parallel IMU callbacks could otherwise corrupt
  // the accumulator sums or double-finalize the calibration. Locked only
  // while !imu_calibrated_ (startup); steady state never touches it.
  std::mutex calib_mtx_;
  // [P2 FIX 2026-07-09] Serializes the first-GT-message extrinsic cache +
  // odom-init block in callbackGtOdom (Reentrant group: two 100 Hz callbacks
  // can run concurrently — std::string assignment to gt_body_frame_ was UB,
  // and applyInitialPose could run twice, interleaved). Leaf-only from the
  // GT thread; never taken while holding pose/geo, never held by anyone who
  // calls back into pose/geo holders.
  std::mutex gt_init_mtx_;
  // [P2 FIX 2026-07-09] Owner lock for the scan-chain seed (basePose,
  // base_pose_stamp_, prev_vel): held by the scan thread across the whole
  // deskew phase, and by the cross-thread reinit writers (applyInitialPose /
  // param pose / RTK full seed) around their seed writes — so a mid-run
  // re-initialization lands atomically BETWEEN scans instead of tearing a
  // quaternion under an in-flight deskew.
  std::mutex seed_mtx_;
  double imu_calib_time_; // seconds to accumulate for calibration
  double imu_calib_start_stamp_;
  int imu_calib_count_;
  Eigen::Vector3f imu_calib_gyro_sum_;
  Eigen::Vector3f imu_calib_accel_sum_;
  // [P3 FIX 2026-07-14] Per-axis sum of squares for the stationarity sanity
  // check (variance = sq_sum/n - mean^2) before baking a stationary gyro bias.
  Eigen::Vector3f imu_calib_gyro_sq_sum_ = Eigen::Vector3f::Zero();

  // RTK-driven IMU calibration (uses GT odom as truth source; allows
  // calibrating while moving). Falls back to the stationary path above if no GT
  // sample arrives within rtk_fallback_timeout_sec_ of the first IMU message.
  enum class InitPhase {
    WAITING,
    RTK_CALIBRATING,
    STATIONARY_CALIBRATING,
    DONE
  };
  std::atomic<InitPhase> init_phase_{InitPhase::WAITING};
  bool rtk_init_enabled_;
  double rtk_calib_window_sec_;
  double rtk_fallback_timeout_sec_;
  double first_imu_stamp_;       // stamp of the first IMU msg (set on receipt)
  double rtk_calib_start_stamp_; // stamp of the first IMU sample paired with GT
  int rtk_calib_count_;
  Eigen::Vector3f rtk_gyro_bias_sum_;
  Eigen::Vector3f rtk_accel_bias_sum_;
  Eigen::Vector3f rtk_gyro_bias_sq_sum_; // for residual stddev sanity check
  Eigen::Vector3f rtk_accel_bias_sq_sum_;
  bool has_prev_gt_for_accel_;
  double prev_gt_stamp_;
  Eigen::Vector3f prev_v_world_;
  GtSample latest_rtk_seed_; // latest GT sample, used to seed state at finalize
  bool has_latest_rtk_seed_;

  // Pose tracking
  struct Pose {
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
  };
  // Tracked base-frame pose in map.
  Pose basePose;
  // [REVIEW FIX 2026-07-08 P2] The scan-chain integration seed as a matrix.
  // Deskew fallback branches previously used current_pose as T_prior, which
  // BYPASSES the INS heading/pose prior (applyInsHeadingPriorToBasePose()
  // corrects basePose, not current_pose) — exactly during timestamp / IMU
  // health trouble, when the drift-free INS reference matters most.
  Eigen::Matrix4f basePoseMatrix() const {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = this->basePose.q.normalized().toRotationMatrix();
    T.block<3, 1>(0, 3) = this->basePose.p;
    return T;
  }
  Eigen::Vector3f prev_vel;
  // IMU-predicted velocity at the same median point time as T_prior. This is
  // the only velocity allowed to seed the next scan; the asynchronous
  // observer state may already be hundreds of IMU samples further ahead.
  Eigen::Vector3f T_prior_velocity_ = Eigen::Vector3f::Zero();

  // Geometric Observer State
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b; // body frame
    Eigen::Vector3f w; // world frame
  };

  struct Velocity {
    Frames lin; // linear velocity
    Frames ang; // angular velocity
  };

  struct State {
    Eigen::Vector3f p;    // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
    Velocity v;           // velocity
    ImuBias b;            // IMU biases in body frame
  };
  State state;

  // One-shot product output built from an accepted GICP pose at scan time plus
  // only the short observer-relative motion from that scan to the latest IMU.
  // It is consumed by publishPose() once at the accepted scan rate; it is never
  // published by IMU callbacks and therefore cannot become IMU-only output
  // during a GICP failure streak.
  struct LatencyCompensatedOutput {
    bool valid = false;
    bool stationary = false;
    uint64_t observer_epoch = 0;
    double measurement_stamp = 0.0;
    double output_stamp = 0.0;
    std::string status = "not_attempted";
    Eigen::Vector3f measurement_p = Eigen::Vector3f::Zero();
    Eigen::Quaternionf measurement_q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f p = Eigen::Vector3f::Zero();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f v_lin_body = Eigen::Vector3f::Zero();
    Eigen::Vector3f v_ang_body = Eigen::Vector3f::Zero();
  } latency_compensated_output_;

  struct Geo {
    std::atomic<bool> first_opt_done;
    std::mutex mtx;
    uint64_t
        update_seq; // Incremented by updateState; checked by propagateState
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  };
  Geo geo;
  // IMU-rate observer poses in timestamp order. GICP is delayed relative to
  // the live observer; its correction must compare against this history at
  // the scan's median time, never against the independent scan-chain prior.
  ObserverPoseHistory observer_pose_history_;
  // A pose-history epoch changes whenever the live observer is discontinuously
  // corrected, reset, or recovered. A delayed GICP result captured in one
  // epoch may never borrow relative motion from another epoch.
  uint64_t observer_epoch_ = 0;              // guarded by geo.mtx
  uint64_t registration_observer_epoch_ = 0; // captured before each solve

  // Current IMU measurement (for propagateState)
  ImuMeas imu_meas;

  // Sensor Type
  gicp_localizer::SensorType sensor;

  // Frames
  std::string map_frame;
  std::string base_frame;
  std::string imu_frame;
  std::string lidar_frame;

  // WGS84 datum for the always-present NavSatFix product topic. When the map
  // manifest/profile does not provide a datum, the topic publishes NO_FIX
  // rather than inventing geographic coordinates.
  bool navsat_origin_valid_{false};
  navsatfix::EnuOrigin navsat_origin_;

  // Parameters
  std::string map_path_;
  bool require_map_manifest_ = false;
  double map_roll_deg_;
  double map_pitch_deg_;
  double map_yaw_deg_;
  bool imu_only_mode_;
  bool use_odom_init_;
  std::atomic<bool> use_odom_init_applied_{
      false}; // P2 fix: read cross-thread (stationary calib guard)
  bool use_param_initial_pose_;
  std::string initial_pose_frame_; // "lidar" or "base_link"
  bool pending_initial_pose_; // true when initial pose needs conversion via
                              // baselink2lidar_T
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_roll_;
  double initial_pose_pitch_;
  double initial_pose_yaw_;

  // GICP parameters
  int executor_threads_ = 0;
  int gicp_num_threads_;
  int gicp_max_iter_;
  int gicp_max_inner_iter_;
  int gicp_startup_max_iter_;
  double gicp_max_optimization_time_ms_;
  double gicp_finalization_reserve_ms_;
  double gicp_startup_max_optimization_time_ms_;
  double gicp_hard_result_max_age_ms_;
  int gicp_startup_accepted_scans_;
  std::atomic<uint64_t> accepted_gicp_count_{0};
  bool failure_streak_had_timeout_{
      false}; // scan worker only; reset on accept/snap
  int gicp_corr_randomness_;
  double gicp_max_corr_dist_;
  double gicp_transformation_epsilon_;
  double gicp_rotation_epsilon_;
  double gicp_fitness_reject_threshold_;
  bool gicp_reject_large_jumps_;
  double gicp_hessian_cond_max_;
  double gicp_hessian_fitness_warn_;
  double gicp_hessian_trans_warn_m_;
  double gicp_hessian_rot_warn_deg_;

  // P1 gating rework: per-map-normalized fitness gates + degeneracy-aware
  // partial updates (solution remapping) + turn-aware yaw-consistency veto.
  // Rationale/thresholds: docs/action_plan_turn_error_20260704.md.
  bool fitness_baseline_enable_; // maintain rolling-median fitness baseline
  int fitness_baseline_window_;  // ring size (accepted-frame fitness samples)
  int fitness_baseline_min_samples_; // gates stay absolute-only until this many
                                     // samples
  double fitness_baseline_seed_; // expected per-map floor used during warm-up
                                 // (0 = off)
  double fitness_ratio_reject_;  // reject scan when fitness/baseline exceeds
                                 // this (<=0 off)
  bool degen_partial_update_enable_; // project delta instead of binary hessian
                                     // reject
  bool degen_full6d_; // full coupled 6x6 remapping (true) vs 3x3 blockwise
                      // (false)
  double degen_coupling_length_m_; // characteristic lever arm making rad and m
                                   // commensurable (full6d)
  double degen_rel_floor_6d_;      // full6d: eigen-axis degenerate if lambda <
                                   // floor*lambda_max
  double degen_rel_floor_rot_; // blockwise: rot eigen-axis degenerate if lambda
                               // < floor*lambda_max(block)
  double
      degen_rel_floor_trans_; // blockwise: trans eigen-axis degenerate likewise
  bool yaw_gate_enable_;      // turn-aware GICP-vs-IMU yaw consistency veto
  double yaw_gate_max_corr_deg_; // SOFT veto: yaw corr above this AND ratio
                                 // above fitnessRatio
  double
      yaw_gate_fitness_ratio_; // soft-tier arming ratio (low-confidence match)
  double yaw_gate_hard_max_corr_deg_;
  double gicp_rp_hard_max_corr_deg_; // P2 fix 2026-07-10h: rp innovation clamp
                                     // (<=0 off)    // HARD veto: unconditional
                                     // yaw-corr bound (<=0 off) — P1 yaw-safety
  double
      gicp_nonconv_ok_max_trans_m_; // PR#6: max correction for the
                                    // non-converged fitness fallback (<=0 off)
  double
      gicp_nonconv_ok_max_rot_deg_; // PR#6: max rotation for the non-converged
                                    // fitness fallback (<=0 off)
  int gicp_min_correspondences_;    // support gate: min inlier correspondences
                                    // (<=0 off)
  double
      gicp_min_corr_ratio_; // support gate: min inliers/points ratio (<=0 off)
  // Algorithmic yaw-defect fixes (optimizer-level, 2026-07-05):
  std::string gicp_dof_mode_; // "6dof" | "4dof" | "3dof" | "planar" (x/y/yaw)
  int gicp_full6dof_every_n_; // periodic unconstrained scan to re-anchor
                              // roll/pitch (0 = never)
  int dof_scan_counter_ = 0;  // scan counter for the periodic 6dof refresh
  int gicp_planar_warmup_accepted_scans_ = 1;
  double gicp_prior_yaw_info_; // soft in-optimizer yaw prior info (rad^-2, 0 =
                               // off)
  double gicp_prior_rollpitch_info_; // soft in-optimizer roll/pitch prior info
                                     // (rad^-2, 0 = off)
  // VKS division of labor: /vks/imu = propagation/deskew/heading,
  // /vks/filtered_odom = initialization and bounded failure recovery.
  bool ins_prior_enable_;
  double ins_prior_yaw_blend_; // fraction of INS-vs-prior yaw applied per scan
  double ins_prior_max_yaw_step_deg_; // hard cap on the per-scan yaw correction
  double ins_prior_sanity_max_yaw_deg_; // above this, warn and do NOT apply
                                        // (frame/INS fault)
  double ins_prior_pos_blend_; // optional position pull toward INS (0 = off)
  double ins_prior_gicp_position_seed_blend_; // Atlas translation used only as
                                              // GICP initial guess
  double ins_prior_gicp_position_seed_max_step_m_; // cap on that initial-guess
                                                   // translation
  bool ins_prior_require_rtk_;         // only consume RTK-quality samples
  double ins_prior_max_yaw_sigma_deg_; // heading-quality gate on sqrt(cov[35]);
                                       // <=0 disables
  // Optional absolute heading from sensor_msgs/Imu::orientation. Kept
  // separate from ins_prior so use_ins=false can still use the already-fused
  // race_common IMU heading without consuming filtered_odom position.
  bool imu_heading_prior_enable_{false};
  double imu_heading_prior_yaw_blend_{1.0};
  double imu_heading_prior_max_yaw_step_deg_{2.0};
  double imu_heading_prior_sanity_max_yaw_deg_{30.0};
  double imu_heading_prior_max_yaw_sigma_deg_{3.0};
  double imu_heading_prior_max_time_error_s_{0.02};
  double last_ins_yaw_diff_deg_ =
      std::numeric_limits<double>::quiet_NaN(); // diagnostic
  std::deque<double>
      fitness_history_; // accepted-frame fitness ring (scan thread only)

  // Static track-map prior. Official race_common TTL geometry only vetoes
  // unsupported LiDAR candidates; it never injects x/y. In planar mode the
  // static TTL profile supplies internal registration/state height, while a
  // separate initialization datum fixes product-output Z.
  bool track_constraint_enabled_ = false;
  bool track_elevation_enabled_ = false;
  std::string track_ttl_directory_;
  std::string track_trajectory_command_topic_;
  std::unique_ptr<TtlTrackConstraint> track_constraint_;
  bool track_adaptive_lateral_enabled_ = false;
  double track_adaptive_lateral_base_m_ = 0.5;
  double track_adaptive_lateral_speed_distance_ratio_ = 0.0;
  double track_adaptive_lateral_max_m_ = 0.5;
  double track_adaptive_lateral_max_scan_dt_s_ = 0.5;
  double track_along_correction_gain_ = 1.0;
  bool track_adaptive_along_enabled_ = false;
  double track_adaptive_along_low_info_per_corr_ = 2.5;
  double track_adaptive_along_full_info_per_corr_ = 5.0;
  double track_adaptive_along_min_gain_ = 0.0;
  double track_max_along_correction_m_ = 3.0;
  double track_recovery_along_max_raw_correction_m_ = 0.0;
  double track_recovery_along_gain_ = 0.0;
  double track_recovery_along_max_applied_correction_m_ = 0.0;
  double track_lateral_fallback_limit_m_ = 0.0;
  bool track_z_offset_valid_ = false;
  double track_z_offset_m_ = 0.0;
  // Internal planar state height, read by IMU propagation and updated from
  // accepted TTL-height registration seeds.
  // Atomics keep the planar Z lock race-free without adding a 200 Hz mutex.
  std::atomic<bool> track_z_hold_valid_{false};
  std::atomic<double> track_z_hold_m_{0.0};
  // Output-only Z datum. It is intentionally not fed back into registration,
  // deskew, or the geometric observer.
  std::atomic<bool> track_output_z_valid_{false};
  std::atomic<double> track_output_z_m_{0.0};

  // Preprocessing parameters
  double crop_size_;
  double scan_min_range_;
  double scan_max_range_;
  bool vf_use_;
  double vf_res_;
  size_t scan_max_points_ = 0;
  bool scan_point_budget_spatial_balancing_ = false;
  size_t scan_point_budget_azimuth_bins_ = 36;
  size_t scan_point_budget_range_bins_ = 3;

  // IMU and deskewing parameters
  bool deskew_;
  double gravity_;
  int imu_buffer_size_;
  double future_imu_wait_timeout_s_ = 1.0;
  uint64_t future_imu_timeout_dropped_ = 0; // scan-worker owned
  double deskew_knot_interval_s_ = 0.002;
  bool flip_y_;

  // Geometric observer parameters
  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double scan_velocity_feedback_gain_;
  std::string wheel_speed_topic_;
  double wheel_speed_feedback_gain_;
  double wheel_speed_max_age_s_;
  bool wheel_stationary_gate_enabled_ = false;
  double wheel_stationary_enter_speed_mps_ = 0.2;
  double wheel_stationary_exit_speed_mps_ = 0.5;
  bool wheel_stationary_gate_active_ = false; // scan-worker owned
  std::mutex wheel_speed_mtx_;
  // Timestamped body-x speeds. The callback can run ahead while the scan
  // worker is inside GICP, so a single latest-value slot is insufficient.
  std::deque<std::pair<double, double>> wheel_speed_buffer_;

  struct FusionWheelSample {
    double stamp_s{0.0};
    Eigen::Vector2d velocity_body_mps{Eigen::Vector2d::Zero()};
    Eigen::Vector2d variance_mps2{Eigen::Vector2d::Constant(0.04)};
  };
  DelayedGicpFusion delayed_fusion_;
  std::mutex delayed_fusion_mtx_;
  uint64_t delayed_fusion_observer_epoch_ =
      std::numeric_limits<uint64_t>::max();
  std::optional<FusionWheelSample> delayed_fusion_latest_wheel_;
  double delayed_fusion_last_wheel_stamp_s_ = -1.0;
  uint64_t delayed_fusion_resets_ = 0;
  uint64_t delayed_fusion_updates_ = 0;
  uint64_t delayed_fusion_failures_ = 0;
  double geo_Kz_damping_;
  double geo_abias_max_;
  double geo_gbias_max_;
  // Maximum recent IMU interval allowed to advance one accepted GICP pose to
  // its publication time. Without a valid bounded bridge the accepted raw
  // GICP measurement is still published at its measurement timestamp.
  double output_max_imu_delay_compensation_s_ = 0.5;
  bool output_wheel_distance_anchor_enabled_ = false;
  bool output_motion_gate_enabled_ = false;
  double output_motion_gate_wheel_endpoint_max_age_s_ = 0.1;
  double output_motion_gate_distance_abs_tolerance_m_ = 0.75;
  double output_motion_gate_distance_rel_tolerance_ = 0.35;
  double output_motion_gate_reverse_tolerance_m_ = 0.1;
  double output_motion_gate_lateral_abs_tolerance_m_ = 0.35;
  double output_motion_gate_lateral_rel_tolerance_ = 0.15;
  double output_motion_gate_yaw_abs_tolerance_rad_ = 0.0872664626;
  double output_motion_gate_max_yaw_rate_rad_s_ = 2.5;

  // Observer-correction stability bounds (P2#1). The proportional observer
  // applies dt*K corrections; this is forward-Euler and only stable for dt*K
  // < 2. A long scan gap (dropped Luminar frames / high-speed racing) would
  // otherwise inject a huge, unstable correction. Cap the effective timestep,
  // and optionally clamp the per-update position/velocity correction magnitude
  // (0 = clamp disabled).
  double
      geo_observer_dt_max_; // s   — cap on dt used in updateState corrections
  double geo_max_pos_correction_; // m   — clamp per-update position correction
                                  // (0=off)
  double geo_max_vel_correction_; // m/s — clamp per-update velocity correction
                                  // (0=off)
  double geo_max_state_speed_; // m/s — hard physical bound on observer speed
                               // (0=off)
  double geo_max_yaw_correction_deg_; // deg — clamp per-update yaw error before
                                      // gain (0=off) — P1 yaw-safety
  double geo_max_rot_correction_deg_; // deg — clamp per-update total rotation
                                      // error (0=off)

  // Time/speed-based dead-reckoning covariance growth (P3). During GICP loss
  // the reported position sigma grows with elapsed dead-reckon time and
  // distance travelled (speed*time), not the raw missed-scan count. 0 rates
  // disable growth.
  double dr_cov_time_rate_; // m of sigma per second of dead reckoning
  double
      dr_cov_dist_frac_; // m of sigma per metre travelled while dead reckoning

  // Debug-log parameters
  bool debug_jump_log_enabled_;
  bool debug_verbose_scan_log_;
  bool debug_lm_print_;
  double debug_jump_trans_m_;
  double debug_jump_rot_deg_;
  // Speed/scan_dt-aware jump-gate scaling (P2#2). The effective large-jump
  // thresholds grow with how far the IMU prior could have drifted: translation
  // with speed*scan_dt, rotation with scan_dt. 0 scales reproduce the fixed
  // thresholds (debug_jump_trans_m_ / debug_jump_rot_deg_).
  double jump_trans_speed_scale_; // extra trans threshold per (speed*scan_dt)
                                  // metre
  double jump_vertical_m_; // fixed prior-vs-last-good Z bound (<=0 disables)
  double
      jump_rot_dt_scale_deg_; // extra rot threshold (deg) per second of scan_dt
  // P1 yaw-safety: yaw-specific innovation gate (split from total rotation).
  double jump_yaw_max_deg_;      // base yaw budget vs IMU prior (<=0 disables)
  double jump_yaw_dt_scale_deg_; // extra yaw budget per second of scan_dt
  double
      jump_yaw_total_max_deg_; // absolute cap the dt scaling can never exceed
  bool verbose_;

  // Extrinsics
  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 baselink2lidar;
    Eigen::Matrix4f baselink2imu_T;
    Eigen::Matrix4f baselink2lidar_T;
  };
  Extrinsics extrinsics;
  bool extrinsics_cached_; // True once baselink2lidar_T has been populated from
                           // TF
  bool imu_extrinsics_cached_; // True once baselink2imu has been populated from
                               // TF

  // Map target configuration
  double map_voxel_size_ = 0.3; // GICP target-map voxel leaf (m); 0 disables
  rclcpp::TimerBase::SharedPtr input_health_timer_;

  // Optional low-bandwidth track-time tuning. RDE telemetry is only a timing
  // source; it never enters the localization objective as a pose measurement.
  bool track_time_tuning_enabled_{false};
  std::string track_time_telemetry_topic_;
  std::vector<double> track_time_breakpoints_s_;
  std::vector<double> track_time_along_gains_;
  std::vector<double> track_time_max_along_corrections_m_;
  std::vector<double> track_time_max_applied_lateral_m_;
  std::atomic<double> track_time_on_track_s_{-1.0};
  std::atomic<int> track_time_tuning_index_{-1};
};

} // namespace gicp_localizer

#endif // GICP_LOCALIZER__GICP_LOCALIZER_NODE_HPP_
