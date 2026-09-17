#include "gicp_interface/detail/localizer_utils.hpp"

#include <algorithm>
#include <cmath>

gicp_localizer::GicpLocalizer::GicpLocalizer(
    const rclcpp::NodeOptions& options)
    : Node("gicp_interface", options) {

  this->getParams();

  // Initialize flags
  this->initialized = false;
  this->first_imu_received = false;

  // Initialize pose
  this->current_pose = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->last_gicp_pose_ = Eigen::Matrix4f::Identity();
  this->last_gicp_stamp_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->last_gicp_valid_ = false;

  // Initialize IMU buffer
  this->imu_buffer.set_capacity(this->imu_buffer_size_);

  // Initialize IMU calibration state
  this->imu_calibrated_ = this->imu_precalibrated_;
  this->imu_calib_start_stamp_ = -1.0;
  this->imu_calib_count_ = 0;
  this->imu_calib_gyro_sum_ = Eigen::Vector3f::Zero();
  this->imu_calib_accel_sum_ = Eigen::Vector3f::Zero();
  this->imu_calib_gyro_sq_sum_ = Eigen::Vector3f::Zero();

  // Initialize RTK-driven calibration state
  this->init_phase_ = this->imu_precalibrated_ ? InitPhase::DONE
                                               : InitPhase::WAITING;
  this->first_imu_stamp_ = -1.0;
  this->rtk_calib_start_stamp_ = -1.0;
  this->rtk_calib_count_ = 0;
  this->rtk_gyro_bias_sum_ = Eigen::Vector3f::Zero();
  this->rtk_accel_bias_sum_ = Eigen::Vector3f::Zero();
  this->rtk_gyro_bias_sq_sum_ = Eigen::Vector3f::Zero();
  this->rtk_accel_bias_sq_sum_ = Eigen::Vector3f::Zero();
  this->has_prev_gt_for_accel_ = false;
  this->prev_gt_stamp_ = 0.0;
  this->prev_v_world_ = Eigen::Vector3f::Zero();
  this->has_latest_rtk_seed_ = false;

  // Initialize previous scan stamp
  this->prev_scan_stamp = 0.0;
  this->base_pose_stamp_ = 0.0;
  this->t_prior_stamp_ = 0.0;
  this->observer_dt_ = 0.0;
  this->last_scan_input_frame_.clear();
  this->last_raw_point_count_ = 0;
  this->last_preprocessed_point_count_ = 0;

  // Initialize lidar pose
  this->basePose.p = Eigen::Vector3f::Zero();
  this->basePose.q = Eigen::Quaternionf::Identity();

  // Initialize previous velocity
  this->prev_vel = Eigen::Vector3f::Zero();
  this->T_prior_velocity_ = Eigen::Vector3f::Zero();

  // Initialize geometric observer state
  this->state.p = Eigen::Vector3f::Zero();
  this->state.q = Eigen::Quaternionf::Identity();
  this->state.v.lin.b = Eigen::Vector3f::Zero();
  this->state.v.lin.w = Eigen::Vector3f::Zero();
  this->state.v.ang.b = Eigen::Vector3f::Zero();
  this->state.v.ang.w = Eigen::Vector3f::Zero();
  this->state.b.gyro = Eigen::Vector3f::Zero();
  this->state.b.accel = Eigen::Vector3f::Zero();
  this->latency_compensated_output_ = {};
  this->observer_epoch_ = 0;
  this->registration_observer_epoch_ = 0;

  this->geo.first_opt_done = false;
  this->geo.update_seq = 0;
  this->consecutive_failures_ = 0;
  this->failure_streak_had_timeout_ = false;
  this->previous_failure_track_along_ = false;
  this->gt_extrinsics_cached_ = false;
  this->T_base_gtbody_.setIdentity();
  this->geo.dp = 0.0;
  this->geo.dq_deg = 0.0;
  this->geo.prev_p = Eigen::Vector3f::Zero();
  this->geo.prev_q = Eigen::Quaternionf::Identity();
  this->geo.prev_vel = Eigen::Vector3f::Zero();

  // [P1 FIX 2026-07-10] The "default to OUSTER" assignment that used to sit
  // here ran AFTER getParams() and unconditionally clobbered the configured
  // localization/sensor_type — with sensor_type=luminar the node logged
  // "Sensor type: luminar" and then silently ran as OUSTER: the Luminar
  // UINT8[8] epoch-ns per-point decode never engaged, every sweep collapsed
  // to a rigid transform (scan_time_span_s == 0.0 on all frames of the
  // 2026-07-09 run3/run5 replays), and mergeAuxClouds treated absolute aux
  // timestamps as relative. Present since the original 2026-03-21 import —
  // ALL prior replays ran without effective per-point deskew. getParams()
  // (called at the top of this constructor) sets `sensor` on every path,
  // including an UNKNOWN fallback with a warning, so no default is needed
  // here at all.

  // Initialize extrinsics to identity (should be configured from parameters)
  this->extrinsics.baselink2imu.t = Eigen::Vector3f::Zero();
  this->extrinsics.baselink2imu.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2lidar.t = Eigen::Vector3f::Zero();
  this->extrinsics.baselink2lidar.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();
  this->extrinsics_cached_ = false;
  this->imu_extrinsics_cached_ = false;
  this->pending_initial_pose_ = false;

  // Initialize point clouds
  this->map_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  this->current_scan = std::make_shared<pcl::PointCloud<PointType>>();
  this->original_scan = std::make_shared<pcl::PointCloud<PointType>>();

  // Load map
  if (!this->loadMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map! Exiting...");
    throw std::runtime_error("Failed to load map");
  }

  // Setup GICP
  // One parameter controls both the explicit small_gicp worker count and the
  // OpenMP deskew loop. Do not silently expand to every core on the vehicle.
  omp_set_dynamic(0);
  omp_set_num_threads(this->gicp_num_threads_);
  this->gicp.setNumThreads(this->gicp_num_threads_);
  this->gicp.setCorrespondenceRandomness(this->gicp_corr_randomness_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setMaximumInnerIterations(this->gicp_max_inner_iter_);
  this->gicp.setMaximumOptimizationTimeMs(this->gicp_max_optimization_time_ms_);
  this->gicp.setFinalizationReserveMs(this->gicp_finalization_reserve_ms_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_epsilon_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_epsilon_);
  this->gicp.setDebugPrint(this->debug_lm_print_);

  // Set target (map)
  this->gicp.setInputTarget(this->map_cloud);
  if (!this->gicp.calculateTargetCovariances()) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to calculate map covariances! GICP will not work correctly.");
    throw std::runtime_error("Failed to calculate target covariances");
  }
  this->initializeLocalMapTarget();

  // Setup subscribers
  this->pointcloud_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto pointcloud_sub_opt = rclcpp::SubscriptionOptions();
  pointcloud_sub_opt.callback_group = this->pointcloud_cb_group;
  // One fixed DDS contract for every LiDAR subscription. SensorDataQoS is
  // BEST_EFFORT, volatile, and keep-last(5); no runtime QoS knobs are exposed.
  const auto lidar_qos = rclcpp::SensorDataQoS();
  RCLCPP_INFO(this->get_logger(),
              "LiDAR subscriptions QoS: fixed SensorDataQoS "
              "(BEST_EFFORT, VOLATILE, keep-last=5)");
  this->pointcloud_sub =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "pointcloud", lidar_qos,
          std::bind(&gicp_localizer::GicpLocalizer::callbackPointCloud, this,
                    std::placeholders::_1),
          pointcloud_sub_opt);

  if (this->track_time_tuning_enabled_ &&
      !this->track_time_telemetry_topic_.empty()) {
    this->track_timing_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    auto track_timing_sub_opt = rclcpp::SubscriptionOptions();
    track_timing_sub_opt.callback_group = this->track_timing_cb_group_;
    const auto track_timing_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    this->track_timing_sub_ =
        this->create_subscription<race_msgs::msg::RdeTelemetry>(
            this->track_time_telemetry_topic_, track_timing_qos,
            [this](race_msgs::msg::RdeTelemetry::ConstSharedPtr msg) {
              const double time_on_track = msg->laps.time_on_track;
              if (std::isfinite(time_on_track) && time_on_track >= 0.0) {
                this->track_time_on_track_s_.store(
                    time_on_track, std::memory_order_release);
              }
            },
            track_timing_sub_opt);
    RCLCPP_INFO(this->get_logger(),
                "Track-time tuning input: %s (RDE laps.time_on_track)",
                this->track_timing_sub_->get_topic_name());
  }

  if (this->track_constraint_enabled_ && this->track_constraint_ &&
      !this->track_trajectory_command_topic_.empty()) {
    this->track_command_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto track_command_sub_opt = rclcpp::SubscriptionOptions();
    track_command_sub_opt.callback_group = this->track_command_cb_group_;
    const auto track_command_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    this->track_trajectory_command_sub_ =
        this->create_subscription<race_msgs::msg::TargetTrajectoryCommand>(
            this->track_trajectory_command_topic_, track_command_qos,
            [this](
                race_msgs::msg::TargetTrajectoryCommand::ConstSharedPtr msg) {
              const int requested = static_cast<int>(msg->current_ttl_index);
              const int previous = this->track_constraint_->activeLineIndex();
              if (!this->track_constraint_->setActiveLineIndex(requested)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Ignoring trajectory command TTL index %d: no matching "
                    "CSV is loaded from '%s'",
                    requested, this->track_ttl_directory_.c_str());
                return;
              }
              if (requested != previous) {
                RCLCPP_INFO(this->get_logger(),
                            "TTL target switched by race_common command: "
                            "%d -> %d (previous line retained until the "
                            "commanded centerline is acquired)",
                            previous, requested);
              }
            },
            track_command_sub_opt);
    RCLCPP_INFO(this->get_logger(),
                "Dynamic TTL selection topic: %s (fail-closed until first "
                "valid command)",
                this->track_trajectory_command_sub_->get_topic_name());
  }

  this->initial_pose_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = this->initial_pose_cb_group;
  this->initial_pose_sub =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
          std::bind(&gicp_localizer::GicpLocalizer::callbackInitialPose, this,
                    std::placeholders::_1),
          initial_pose_sub_opt);

  // Aux LiDAR subscribers (multi-LiDAR concatenation). Use a Reentrant group so
  // aux scans can land in parallel with the primary callback and with each
  // other; each aux only writes to its own buffer (mutex-protected), so no
  // shared mutable state is touched here.
  if (this->concat_enabled_ && !this->aux_lidars_.empty()) {
    this->aux_cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto aux_sub_opt = rclcpp::SubscriptionOptions();
    aux_sub_opt.callback_group = this->aux_cb_group_;
    // [P3 FIX 2026-07-14] Reject an aux topic that resolves to the same topic
    // as the primary cloud: the same message would be delivered to BOTH the
    // primary and aux callbacks, which the front/aux synchronizer cannot
    // reconcile (parity with the offline readers, where it desynchronizes the
    // two-pass ordinal plan and maps zero scans). Compare RESOLVED names so a
    // remap collision is caught too. Fail loud at construction.
    const std::string primary_resolved = this->pointcloud_sub->get_topic_name();
    for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
      const int idx = static_cast<int>(i);
      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          this->aux_lidars_[i]->topic, lidar_qos,
          [this, idx](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            this->callbackAuxPointCloud(idx, std::move(msg));
          },
          aux_sub_opt);
      if (std::string(sub->get_topic_name()) == primary_resolved) {
        RCLCPP_FATAL(
            this->get_logger(),
            "lidar_concat: aux topic '%s' resolves to the primary cloud topic "
            "'%s' — "
            "the same cloud would feed both primary and aux; refusing to start",
            this->aux_lidars_[i]->topic.c_str(), primary_resolved.c_str());
        throw std::runtime_error(
            "lidar_concat: aux topic collides with the primary cloud topic");
      }
      this->aux_subs_.push_back(sub);
      RCLCPP_INFO(this->get_logger(), "Subscribed to aux LiDAR topic: %s",
                  sub->get_topic_name());
    }
  }

  // [REVIEW FIX 2026-07-08 P1] MutuallyExclusive, not Reentrant. IMU still
  // runs in PARALLEL with pointcloud processing (different callback groups) —
  // exclusivity only serializes IMU-vs-IMU. Reentrant IMU callbacks could
  // interleave so an older callback propagated a newer sample (or two
  // callbacks propagated the same one), and the propagateState() commit
  // guard (geo.update_seq, which IMU commits do not advance) let the older
  // propagation overwrite the newer state. Serial in-order IMU processing
  // removes the entire class: monotonic buffer, one propagation per sample,
  // ordered commits, and the function-local static counters become safe.
  // Cost: none in practice — propagateState is O(100 us) at ~100 Hz.
  this->imu_cb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group;

  // Set QoS for IMU subscriber (BEST_EFFORT to match sensor publishers)
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
  imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", imu_qos,
      std::bind(&gicp_localizer::GicpLocalizer::callbackImu, this,
                std::placeholders::_1),
      imu_sub_opt);

  // Optional proprioceptive longitudinal-speed anchor. race_common exposes
  // its wheel model as TwistWithCovarianceStamped on /vks/wheels; unlike
  // filtered_odom this contributes no absolute position or heading.
  if (!this->wheel_speed_topic_.empty()) {
    this->wheel_speed_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto wheel_sub_opt = rclcpp::SubscriptionOptions();
    wheel_sub_opt.callback_group = this->wheel_speed_cb_group_;
    this->wheel_speed_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        this->wheel_speed_topic_, rclcpp::SensorDataQoS().keep_last(200),
        [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr
                   msg) {
          const double stamp = rclcpp::Time(msg->header.stamp).seconds();
          const double speed = msg->twist.twist.linear.x;
          const double lateral_speed = msg->twist.twist.linear.y;
          if (!std::isfinite(stamp) || !std::isfinite(speed) || speed < 0.0 ||
              !std::isfinite(lateral_speed)) {
            return;
          }
          {
            std::lock_guard<std::mutex> lock(this->wheel_speed_mtx_);
            if (!this->wheel_speed_buffer_.empty() &&
                stamp < this->wheel_speed_buffer_.back().first) {
              return;
            }
            this->wheel_speed_buffer_.emplace_back(stamp, speed);
            while (!this->wheel_speed_buffer_.empty() &&
                   stamp - this->wheel_speed_buffer_.front().first > 5.0) {
              this->wheel_speed_buffer_.pop_front();
            }
          }
          constexpr double kWheelVarianceFloorMps2 = 0.04;
          Eigen::Vector2d variance(msg->twist.covariance[0],
                                   msg->twist.covariance[7]);
          for (int axis = 0; axis < 2; ++axis) {
            if (!std::isfinite(variance(axis)) || variance(axis) <= 0.0) {
              variance(axis) = kWheelVarianceFloorMps2;
            } else {
              variance(axis) =
                  std::max(variance(axis), kWheelVarianceFloorMps2);
            }
          }
          this->updateDelayedFusionWithWheel(
              stamp, Eigen::Vector2d(speed, lateral_speed), variance);
        },
        wheel_sub_opt);
    RCLCPP_INFO(this->get_logger(), "Wheel-speed input topic: %s",
                this->wheel_speed_sub_->get_topic_name());
  }
  const std::string resolved_imu_topic = this->imu_sub->get_topic_name();
  if (this->imu_require_topic_allowlist_) {
    const bool allowed =
        std::find(this->imu_topic_allowlist_.begin(),
                  this->imu_topic_allowlist_.end(),
                  resolved_imu_topic) != this->imu_topic_allowlist_.end();
    if (!allowed) {
      std::ostringstream oss;
      for (size_t i = 0; i < this->imu_topic_allowlist_.size(); ++i) {
        if (i)
          oss << ", ";
        oss << this->imu_topic_allowlist_[i];
      }
      RCLCPP_FATAL(this->get_logger(),
                   "IMU topic hard guard triggered: resolved topic '%s' is not "
                   "in configured allowlist [%s].",
                   resolved_imu_topic.c_str(), oss.str().c_str());
      throw std::runtime_error("IMU topic hard guard mismatch");
    }
  }
  RCLCPP_INFO(this->get_logger(),
              "IMU input topic: %s (configured frame='%s', "
              "strict_frame_match=%s)",
              resolved_imu_topic.c_str(), this->imu_frame.c_str(),
              this->imu_require_frame_match_ ? "true" : "false");

  // IMU input health check. The most common silent failure is launching with an
  // `imu_topic:=` arg that doesn't match any publisher — the subscription is
  // created but callbacks never fire and there's nothing in the log to tell
  // the user why. Fire a periodic timer that warns when no IMU has been
  // received AND no publisher exists on the resolved topic name. The timer
  // cancels itself once the first IMU arrives.
  this->input_health_timer_ =
      this->create_wall_timer(std::chrono::seconds(3), [this]() {
        if (this->first_imu_received.load()) {
          this->input_health_timer_->cancel();
          return;
        }
        const std::string imu_topic = this->imu_sub->get_topic_name();
        const size_t pub_count = this->count_publishers(imu_topic);
        if (pub_count == 0) {
          RCLCPP_WARN(this->get_logger(),
                      "No IMU received on '%s' (0 publishers). Check the "
                      "imu_topic launch arg. "
                      "Run `ros2 topic list | grep -i imu` to see available "
                      "IMU topics and verify localization/imu/topic_allowlist.",
                      imu_topic.c_str());
        } else {
          RCLCPP_WARN(
              this->get_logger(),
              "No IMU received on '%s' yet, but %zu publisher(s) exist. "
              "QoS mismatch or sim-time/clock issue is possible.",
              imu_topic.c_str(), pub_count);
        }
      });

  // Optional reference-odom subscriber for init, recovery, calibration, and
  // the INS heading prior.
  // Topic is remappable as "gt_odom"; the vehicle launch defaults to VKS'
  // selected, CG-referenced /vks/filtered_odom stream.
  if (this->gt_odom_enabled_) {
    this->gt_odom_cb_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto gt_sub_opt = rclcpp::SubscriptionOptions();
    gt_sub_opt.callback_group = this->gt_odom_cb_group;
    auto gt_qos = rclcpp::QoS(rclcpp::KeepLast(this->gt_odom_buffer_size_));
    gt_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    gt_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    this->gt_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "gt_odom", gt_qos,
        std::bind(&gicp_localizer::GicpLocalizer::callbackGtOdom, this,
                  std::placeholders::_1),
        gt_sub_opt);
    RCLCPP_INFO(this->get_logger(),
                "Reference odom enabled (topic remap 'gt_odom', "
                "buffer=%zu, max_dt=%.3fs)",
                this->gt_odom_buffer_size_, this->gt_odom_max_dt_);
  }

  // RTK quality gate is now self-contained in callbackGtOdom: it inspects
  // msg->pose.covariance on each gt_odom sample (no separate subscription).
  // Log the current configuration so the operator can see what's active.
  if (this->gt_odom_enabled_) {
    if (this->rtk_gate_enabled_) {
      RCLCPP_INFO(this->get_logger(),
                  "Reference-odom quality gate ENABLED (per-consumer): all "
                  "gt_odom samples are buffered; bias calibration, the INS "
                  "heading prior use only samples "
                  "with finite, nonnegative pose covariance within "
                  "max_pose_var_xy=%.3f m^2 / max_pose_var_z=%.3f m^2. Snap "
                  "recovery and odom-init accept degraded samples by design.",
                  this->rtk_gate_max_pose_var_xy_,
                  this->rtk_gate_max_pose_var_z_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "RTK quality gate DISABLED: bias calibration, the INS "
                  "heading prior will consume "
                  "gt_odom samples regardless of reported covariance. "
                  "(Buffering, snap recovery, and odom-init are unaffected "
                  "either way.) Enable localization/rtk_gate/enable to "
                  "restrict those consumers to RTK-quality samples.");
    }
  }

  // Setup the two fixed product publishers.
  if (this->imu_only_mode_) {
    RCLCPP_INFO(
        this->get_logger(),
        "Odometry output: IMU-only diagnostic mode (GICP disabled)");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Odometry output: accepted scan-rate GICP solutions only");
  }

  // The canonical output is scan-rate in normal mode and IMU-rate only when
  // the explicit diagnostic imu_only mode is requested.
  auto output_qos = rclcpp::QoS(rclcpp::KeepLast(100));
  output_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  output_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  this->localized_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
      "/localization/gicp/odom", output_qos);
  this->navsatfix_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/localization/gicp/navsatfix", output_qos);

  // TF buffer and listener for transforming incoming point clouds
  this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

  RCLCPP_INFO(this->get_logger(), "DLIO Localization Node Initialized");
  RCLCPP_INFO(this->get_logger(), "Map loaded with %lu points",
              this->map_cloud->points.size());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  this->applyInitialPoseFromParams();

  // Async Luminar front worker. The subscription callback only validates and
  // enqueues; this worker owns processing order. This is required even for the
  // production front-only localizer: a single slow GICP iteration must not
  // block the DDS callback long enough for keep-last history to discard later
  // 10 Hz clouds silently. With aux LiDARs enabled, the same worker also owns
  // the point-time synchronizer wait. Non-Luminar sensors keep the legacy
  // synchronous path.
  // Started LAST, after every throwing constructor step: if the constructor
  // throws after a thread starts, the destructor never runs and the thread
  // would touch destroyed members. Callbacks cannot fire before spin, so no
  // front cloud can arrive between subscription creation and this point.
  if (!this->imu_only_mode_ &&
      this->sensor == gicp_localizer::SensorType::LUMINAR) {
    this->sync_active_ = true;
    this->sync_worker_ =
        std::thread(&gicp_localizer::GicpLocalizer::syncWorkerLoop, this);
    RCLCPP_INFO(
        this->get_logger(),
        "front sync: async Luminar worker active "
        "(aux=%zu, point gate %.3fs, deadline %.3fs, primary queue %zu, "
        "registration rate %.3f Hz, OpenMP/GICP threads %d)",
        this->aux_lidars_.size(), this->concat_luminar_point_threshold_,
        this->concat_future_aux_wait_s_, this->concat_primary_queue_size_,
        this->registration_rate_hz_, this->gicp_num_threads_);
  }

  this->start();
  // Raw `this` is safe: the handle is removed in the destructor, so the
  // callback can never outlive the node.
  this->pre_shutdown_handle_ =
      this->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
          [this]() { this->drainFrontSync(); });
}

gicp_localizer::GicpLocalizer::~GicpLocalizer() {
  auto context = this->get_node_base_interface()->get_context();
  if (context->is_valid()) {
    context->remove_pre_shutdown_callback(this->pre_shutdown_handle_);
  }
  // Fallback only: the pre-shutdown callback in localization_node.cc calls
  // drainFrontSync() while the context is still valid, so queued fronts are
  // PROCESSED. Reaching this with a live worker means the run tail can only
  // be accounted (rclcpp::ok() is already false here on the normal path).
  this->drainFrontSync();
  this->stopLocalMapBuilder();
}

void gicp_localizer::GicpLocalizer::drainFrontSync() {
  // Never join from the worker thread itself: the fatal path calls
  // rclcpp::shutdown() ON the worker, which invokes the pre-shutdown callback
  // synchronously — a self-join would throw. Main/destructor drains later.
  if (this->sync_worker_.get_id() == std::this_thread::get_id()) {
    return;
  }
  std::lock_guard<std::mutex> dlk(this->drain_mtx_); // idempotent/thread-safe
  if (!this->sync_worker_.joinable()) {
    return;
  }
  // Set shutdown UNDER sync_mtx_: the worker holds sync_mtx_ continuously
  // from its predicate check to entering the wait, so a store+notify under
  // the same mutex can never land in that gap and be lost (the former
  // lock-free store could, deadlocking this join forever).
  {
    std::lock_guard<std::mutex> lk(this->sync_mtx_);
    this->sync_shutdown_.store(true);
  }
  this->sync_cv_.notify_all();
  this->sync_worker_.join(); // worker drains remaining fronts in order

  // End-of-run conservation summary. The invariant has no aux term because
  // no code path drops a front for aux reasons (front_dropped_due_to_aux is
  // structurally zero); overload_dropped is the only drop and it is counted.
  //
  // [P3 FIX 2026-07-14] Snapshot the counters UNDER sync_mtx_. The worker is
  // joined, but the subscription is still live: a late callbackPointCloud ->
  // enqueuePrimary can still increment these counters concurrently, so the
  // former unlocked reads were a data race and could compute a spurious
  // "conservation VIOLATED". enqueuePrimary now accounts post-shutdown fronts
  // as shutdown_unprocessed (rather than orphaning them in primary_queue_), so
  // received == accounted holds at every instant and a locked snapshot is
  // self-consistent. Any front still queued here (worker exited before
  // draining it) is also folded into shutdown_unprocessed.
  uint64_t received, released, invalid, rate_limited, shutdown_unprocessed,
      overload_dropped, epoch_dropped, registration_timeout_dropped,
      strict_sync_skipped, epoch_inflight_dropped;
  uint64_t c_all, c_wm, c_to, c_sd, c_noabs;
  bool fatal;
  {
    std::lock_guard<std::mutex> lk(this->sync_mtx_);
    while (!this->primary_queue_.empty()) {
      this->primary_queue_.pop_front();
      ++this->front_shutdown_unprocessed_;
    }
    received = this->front_received_;
    released = this->front_released_;
    invalid = this->front_invalid_;
    rate_limited = this->front_rate_limited_;
    shutdown_unprocessed = this->front_shutdown_unprocessed_;
    overload_dropped = this->front_overload_dropped_;
    epoch_dropped = this->front_epoch_dropped_;
    registration_timeout_dropped = this->front_registration_timeout_dropped_;
    strict_sync_skipped = this->front_strict_sync_skipped_;
    epoch_inflight_dropped = this->front_epoch_inflight_dropped_;
    c_all = this->release_reason_counts_[RELEASE_ALL_MATCHED];
    c_wm = this->release_reason_counts_[RELEASE_WATERMARK];
    c_to = this->release_reason_counts_[RELEASE_TIMEOUT];
    c_sd = this->release_reason_counts_[RELEASE_SHUTDOWN_DRAIN];
    c_noabs = this->release_reason_counts_[RELEASE_PRIMARY_NO_ABSTIME];
    fatal = this->sync_fatal_.load();
  }
  const uint64_t accounted = released + invalid + rate_limited +
                             shutdown_unprocessed + overload_dropped +
                             epoch_dropped + registration_timeout_dropped;
  RCLCPP_INFO(
      this->get_logger(),
      "front sync summary: received=%lu released=%lu invalid=%lu "
      "rate_limited=%lu "
      "shutdown_unprocessed=%lu overload_dropped=%lu epoch_dropped=%lu "
      "registration_timeout_dropped=%lu strict_sync_skipped=%lu "
      "epoch_inflight_dropped=%lu | releases: "
      "all_matched=%lu watermark=%lu timeout=%lu shutdown_drain=%lu "
      "primary_no_abstime=%lu%s",
      static_cast<unsigned long>(received),
      static_cast<unsigned long>(released), static_cast<unsigned long>(invalid),
      static_cast<unsigned long>(rate_limited),
      static_cast<unsigned long>(shutdown_unprocessed),
      static_cast<unsigned long>(overload_dropped),
      static_cast<unsigned long>(epoch_dropped),
      static_cast<unsigned long>(registration_timeout_dropped),
      static_cast<unsigned long>(strict_sync_skipped),
      static_cast<unsigned long>(epoch_inflight_dropped),
      static_cast<unsigned long>(c_all), static_cast<unsigned long>(c_wm),
      static_cast<unsigned long>(c_to), static_cast<unsigned long>(c_sd),
      static_cast<unsigned long>(c_noabs),
      fatal ? " | FATAL pipeline error occurred" : "");
  if (received != accounted) {
    RCLCPP_ERROR(
        this->get_logger(),
        "front sync: conservation VIOLATED (received=%lu != accounted=%lu) — "
        "a front cloud was lost on an unaccounted path",
        static_cast<unsigned long>(received),
        static_cast<unsigned long>(accounted));
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gicp_localizer::GicpLocalizer)
