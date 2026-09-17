#include "gicp_interface/detail/localizer_utils.hpp"

#include <algorithm>

namespace {

bool parseEnuOrigin(const std::string &value,
                    gicp_localizer::navsatfix::EnuOrigin &origin) {
  std::string normalized = value;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::istringstream stream(normalized);
  if (!(stream >> origin.latitude_deg >> origin.longitude_deg >>
        origin.altitude_m) || origin.latitude_deg < -90.0 ||
      origin.latitude_deg > 90.0 || origin.longitude_deg < -180.0 ||
      origin.longitude_deg > 180.0 || !std::isfinite(origin.latitude_deg) ||
      !std::isfinite(origin.longitude_deg) ||
      !std::isfinite(origin.altitude_m)) {
    return false;
  }
  std::string extra;
  return !(stream >> extra);
}

} // namespace

void gicp_localizer::GicpLocalizer::getParams() {

  // Frame IDs
  this->declare_parameter<std::string>("localization/map_frame", "map");
  this->declare_parameter<std::string>("localization/base_frame", "base_link");
  this->declare_parameter<std::string>("localization/imu_frame", "imu");
  this->declare_parameter<std::string>("localization/lidar_frame", "lidar");
  // Static base_frame<-lidar_frame lever arm (row-major 4x4) used to resolve
  // the extrinsic WITHOUT live TF in offline replay. Empty = rely on URDF
  // (lidar_concat/urdf_path) then live TF. See
  // resolveBaseLidarExtrinsicOffline().
  this->declare_parameter<std::vector<double>>(
      "localization/base_lidar_transform", std::vector<double>{});
  this->declare_parameter<std::string>(
      "localization/base_lidar_transform_frame", "");

  this->get_parameter("localization/map_frame", this->map_frame);
  this->get_parameter("localization/base_frame", this->base_frame);
  this->get_parameter("localization/imu_frame", this->imu_frame);
  this->get_parameter("localization/lidar_frame", this->lidar_frame);
  this->get_parameter("localization/base_lidar_transform",
                      this->base_lidar_static_);
  this->get_parameter("localization/base_lidar_transform_frame",
                      this->base_lidar_static_frame_);
  if (this->base_lidar_static_frame_.empty()) {
    // Backward compatibility for profiles created before the transform was
    // explicitly bound to its input frame.
    this->base_lidar_static_frame_ = this->lidar_frame;
  }

  // Map parameters
  this->declare_parameter<std::string>("localization/map_path", "");
  this->declare_parameter<bool>("localization/require_map_manifest", false);
  // [P3 FIX 2026-07-10] optional ENU-datum enforcement against the map manifest
  this->declare_parameter<std::string>("localization/expected_enu_origin", "");
  // Voxel leaf size (m) for the GICP TARGET map / kd-tree. A dense map (e.g. a
  // 49M-point GLIM export) builds a huge kd-tree -> >10 GiB RSS and swap thrash
  // that stalls registration. Downsampling the target to ~0.3 m cuts memory and
  // per-scan search cost with negligible accuracy loss at the matching 0.3 m
  // scan voxel (scan_downsample).
  // 0.0 disables (use the full-resolution map).
  this->declare_parameter<double>("localization/map_voxel_size", 0.3);
  this->declare_parameter<bool>("localization/local_map/enabled", false);
  this->declare_parameter<double>("localization/local_map/radius_m", 200.0);
  this->declare_parameter<double>(
      "localization/local_map/rebuild_distance_m", 40.0);
  this->declare_parameter<double>("localization/local_map/lead_time_s", 0.5);
  this->declare_parameter<double>("localization/local_map/max_lead_m", 25.0);
  this->declare_parameter<double>(
      "localization/local_map/grid_cell_size_m", 50.0);
  this->declare_parameter<int>("localization/local_map/min_points", 10000);
  this->declare_parameter<int>("localization/local_map/builder_threads", 1);
  this->declare_parameter<double>("localization/map_rotation/roll_deg", 0.0);
  this->declare_parameter<double>("localization/map_rotation/pitch_deg", 0.0);
  this->declare_parameter<double>("localization/map_rotation/yaw_deg", 0.0);

  this->get_parameter("localization/map_path", this->map_path_);
  this->get_parameter("localization/require_map_manifest",
                      this->require_map_manifest_);
  std::string expected_enu_origin;
  this->get_parameter("localization/expected_enu_origin", expected_enu_origin);
  if (!expected_enu_origin.empty()) {
    if (!parseEnuOrigin(expected_enu_origin, this->navsat_origin_)) {
      throw std::invalid_argument(
          "localization/expected_enu_origin must be 'lat_deg,lon_deg,alt_m'");
    }
    this->navsat_origin_valid_ = true;
  }
  this->get_parameter("localization/map_voxel_size", this->map_voxel_size_);
  this->get_parameter("localization/local_map/enabled",
                      this->local_map_enabled_);
  this->get_parameter("localization/local_map/radius_m",
                      this->local_map_radius_m_);
  this->get_parameter("localization/local_map/rebuild_distance_m",
                      this->local_map_rebuild_distance_m_);
  this->get_parameter("localization/local_map/lead_time_s",
                      this->local_map_lead_time_s_);
  this->get_parameter("localization/local_map/max_lead_m",
                      this->local_map_max_lead_m_);
  this->get_parameter("localization/local_map/grid_cell_size_m",
                      this->local_map_grid_cell_size_m_);
  int local_map_min_points = 10000;
  this->get_parameter("localization/local_map/min_points",
                      local_map_min_points);
  this->get_parameter("localization/local_map/builder_threads",
                      this->local_map_builder_threads_);
  if (!std::isfinite(this->local_map_radius_m_) ||
      !std::isfinite(this->local_map_rebuild_distance_m_) ||
      !std::isfinite(this->local_map_lead_time_s_) ||
      !std::isfinite(this->local_map_max_lead_m_) ||
      !std::isfinite(this->local_map_grid_cell_size_m_) ||
      this->local_map_radius_m_ <= 0.0 ||
      this->local_map_rebuild_distance_m_ <= 0.0 ||
      this->local_map_lead_time_s_ < 0.0 ||
      this->local_map_max_lead_m_ < 0.0 ||
      this->local_map_grid_cell_size_m_ <= 0.0 ||
      local_map_min_points < 1 || this->local_map_builder_threads_ < 1) {
    throw std::invalid_argument(
        "localization/local_map parameters must be finite and positive "
        "(lead_time/max_lead may be zero)");
  }
  this->local_map_min_points_ = static_cast<size_t>(local_map_min_points);
  this->get_parameter("localization/map_rotation/roll_deg",
                      this->map_roll_deg_);
  this->get_parameter("localization/map_rotation/pitch_deg",
                      this->map_pitch_deg_);
  this->get_parameter("localization/map_rotation/yaw_deg", this->map_yaw_deg_);

  // Localization parameters
  this->declare_parameter<bool>("localization/imu_only", false);
  this->declare_parameter<double>(
      "localization/output/max_imu_delay_compensation_s", 0.5);
  this->declare_parameter<bool>(
      "localization/output/wheel_distance_anchor/enable", false);
  this->declare_parameter<bool>(
      "localization/output/motion_gate/enable", false);
  this->declare_parameter<double>(
      "localization/output/motion_gate/wheel_endpoint_max_age_s", 0.1);
  this->declare_parameter<double>(
      "localization/output/motion_gate/distance_abs_tolerance_m", 0.75);
  this->declare_parameter<double>(
      "localization/output/motion_gate/distance_rel_tolerance", 0.35);
  this->declare_parameter<double>(
      "localization/output/motion_gate/reverse_tolerance_m", 0.1);
  this->declare_parameter<double>(
      "localization/output/motion_gate/lateral_abs_tolerance_m", 0.35);
  this->declare_parameter<double>(
      "localization/output/motion_gate/lateral_rel_tolerance", 0.15);
  this->declare_parameter<double>(
      "localization/output/motion_gate/yaw_abs_tolerance_deg", 5.0);
  this->declare_parameter<double>(
      "localization/output/motion_gate/max_yaw_rate_rad_s", 2.5);
  this->declare_parameter<bool>("localization/use_odom_init", true);
  this->declare_parameter<bool>("localization/initial_pose/use", false);
  this->declare_parameter<std::string>("localization/initial_pose/frame",
                                       "lidar");
  this->declare_parameter<double>("localization/initial_pose/x", 0.0);
  this->declare_parameter<double>("localization/initial_pose/y", 0.0);
  this->declare_parameter<double>("localization/initial_pose/z", 0.0);
  this->declare_parameter<double>("localization/initial_pose/roll", 0.0);
  this->declare_parameter<double>("localization/initial_pose/pitch", 0.0);
  this->declare_parameter<double>("localization/initial_pose/yaw", 0.0);

  // External odom for initialization, recovery, and optional INS priors.
  // Uses topic remap "gt_odom".
  this->declare_parameter<bool>("localization/gt_odom/enable",
                                true); // [P3 FIX 2026-07-10] yaml-aligned
  this->declare_parameter<int>("localization/gt_odom/buffer_size", 200);
  this->declare_parameter<double>("localization/gt_odom/max_dt", 0.1);
  this->declare_parameter<std::string>("localization/gt_odom/expected_frame_id",
                                       this->map_frame);
  this->declare_parameter<std::string>(
      "localization/gt_odom/expected_child_frame_id", this->base_frame);
  this->declare_parameter<std::vector<double>>(
      "ins_offset", std::vector<double>{0.0, 0.0, 0.0});
  // [P2 FIX 2026-07-14] Bracket-width bound for GT interpolation (see
  // getGtPoseAt / getGtFiniteDiffVelWorld). Independent of max_dt, which only
  // bounds the nearer endpoint. Default 0.5 s: at nominal 10 Hz GT this is 5×
  // the sample period, so it never rejects a healthy stream but forbids
  // chording across a real dropout.
  this->declare_parameter<double>("localization/gt_odom/interp_max_gap", 0.5);
  bool gt_enable = false;
  int gt_buf = 200;
  double gt_max_dt = 0.1;
  double gt_interp_gap = 0.5;
  this->get_parameter("localization/gt_odom/enable", gt_enable);
  this->get_parameter("localization/gt_odom/buffer_size", gt_buf);
  this->get_parameter("localization/gt_odom/max_dt", gt_max_dt);
  this->get_parameter("localization/gt_odom/expected_frame_id",
                      this->gt_expected_frame_id_);
  this->get_parameter("localization/gt_odom/expected_child_frame_id",
                      this->gt_expected_child_frame_id_);
  std::vector<double> ins_offset;
  this->get_parameter("ins_offset", ins_offset);
  if (ins_offset.size() != 3 ||
      !std::all_of(ins_offset.begin(), ins_offset.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("ins_offset must contain exactly three finite "
                                "map-frame values [x,y,z]");
  }
  this->ins_offset_ = Eigen::Vector3f(static_cast<float>(ins_offset[0]),
                                      static_cast<float>(ins_offset[1]),
                                      static_cast<float>(ins_offset[2]));
  this->get_parameter("localization/gt_odom/interp_max_gap", gt_interp_gap);
  this->gt_odom_enabled_ = gt_enable;
  this->gt_odom_buffer_size_ = static_cast<size_t>(std::max(gt_buf, 1));
  this->gt_odom_max_dt_ = gt_max_dt;
  // [P3 FIX 2026-07-10] Negative durations invert every |dt| comparison —
  // ALL GT lookups would fail silently (killing snap/calibration/heading prior).
  if (!std::isfinite(this->gt_odom_max_dt_) || this->gt_odom_max_dt_ < 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "localization/gt_odom/max_dt=%.3f invalid; using 0.15",
                this->gt_odom_max_dt_);
    this->gt_odom_max_dt_ = 0.15;
  }
  this->gt_interp_max_gap_ = gt_interp_gap;
  if (!std::isfinite(this->gt_interp_max_gap_) ||
      this->gt_interp_max_gap_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "localization/gt_odom/interp_max_gap=%.3f invalid; using 0.5",
                this->gt_interp_max_gap_);
    this->gt_interp_max_gap_ = 0.5;
  }
  RCLCPP_INFO(this->get_logger(),
              "External INS/reference map offset: [%.3f, %.3f, %.3f] m",
              this->ins_offset_.x(), this->ins_offset_.y(),
              this->ins_offset_.z());

  // Reference-odom quality gate: consumers requiring high-quality reference
  // data reject samples whose position covariance exceeds these thresholds.
  // Conservative
  // defaults: gate ON; xy threshold 0.25 m^2 (~0.5 m std, comfortably above
  // RTK-fixed and float-mode covariances measured on AV-24 ~ 5e-5 m^2);
  // z threshold 1.0 m^2 (~1 m std, since GPS Z is naturally worse).
  this->declare_parameter<bool>("localization/rtk_gate/enable", true);
  this->declare_parameter<bool>("localization/rtk_gate/allow_zero_covariance",
                                false);
  this->declare_parameter<double>("localization/rtk_gate/max_pose_var_xy",
                                  0.25);
  this->declare_parameter<double>("localization/rtk_gate/max_pose_var_z", 1.0);
  this->get_parameter("localization/rtk_gate/enable", this->rtk_gate_enabled_);
  this->get_parameter("localization/rtk_gate/allow_zero_covariance",
                      this->rtk_gate_allow_zero_covariance_);
  this->get_parameter("localization/rtk_gate/max_pose_var_xy",
                      this->rtk_gate_max_pose_var_xy_);
  this->get_parameter("localization/rtk_gate/max_pose_var_z",
                      this->rtk_gate_max_pose_var_z_);

  // GT-driven pose recovery (optional). Independent of gt_odom/enable; recovery
  // requires the same subscriber to be active, so it implies gt_odom/enable.
  this->declare_parameter<bool>("localization/gt_recovery/enable", false);
  // Three consecutive failures trigger one explicit recovery attempt. The
  // snap resets the counter, so another recovery requires three new rejects;
  // healthy GICP scans never consume the external pose here.
  this->declare_parameter<int>(
      "localization/gt_recovery/min_consecutive_failures", 3);
  this->get_parameter("localization/gt_recovery/enable",
                      this->gt_recovery_enabled_);
  this->get_parameter("localization/gt_recovery/min_consecutive_failures",
                      this->gt_recovery_min_consecutive_failures_);
  if (this->gt_recovery_enabled_ && !this->gt_odom_enabled_) {
    RCLCPP_WARN(this->get_logger(),
                "localization/gt_recovery/enable=true but gt_odom/enable=false "
                "— forcing gt_odom on so the buffer fills.");
    this->gt_odom_enabled_ = true;
  }
  if (this->gt_recovery_min_consecutive_failures_ < 1) {
    this->gt_recovery_min_consecutive_failures_ = 1;
  }

  this->get_parameter("localization/imu_only", this->imu_only_mode_);
  this->get_parameter(
      "localization/output/max_imu_delay_compensation_s",
      this->output_max_imu_delay_compensation_s_);
  if (!std::isfinite(this->output_max_imu_delay_compensation_s_) ||
      this->output_max_imu_delay_compensation_s_ <= 0.0) {
    RCLCPP_WARN(
        this->get_logger(),
        "localization/output/max_imu_delay_compensation_s must be finite and "
        "> 0; using 0.5 s");
    this->output_max_imu_delay_compensation_s_ = 0.5;
  }
  this->get_parameter("localization/output/wheel_distance_anchor/enable",
                      this->output_wheel_distance_anchor_enabled_);
  this->get_parameter("localization/output/motion_gate/enable",
                      this->output_motion_gate_enabled_);
  this->get_parameter(
      "localization/output/motion_gate/wheel_endpoint_max_age_s",
      this->output_motion_gate_wheel_endpoint_max_age_s_);
  this->get_parameter(
      "localization/output/motion_gate/distance_abs_tolerance_m",
      this->output_motion_gate_distance_abs_tolerance_m_);
  this->get_parameter(
      "localization/output/motion_gate/distance_rel_tolerance",
      this->output_motion_gate_distance_rel_tolerance_);
  this->get_parameter(
      "localization/output/motion_gate/reverse_tolerance_m",
      this->output_motion_gate_reverse_tolerance_m_);
  this->get_parameter(
      "localization/output/motion_gate/lateral_abs_tolerance_m",
      this->output_motion_gate_lateral_abs_tolerance_m_);
  this->get_parameter(
      "localization/output/motion_gate/lateral_rel_tolerance",
      this->output_motion_gate_lateral_rel_tolerance_);
  double output_motion_gate_yaw_abs_tolerance_deg = 5.0;
  this->get_parameter(
      "localization/output/motion_gate/yaw_abs_tolerance_deg",
      output_motion_gate_yaw_abs_tolerance_deg);
  this->get_parameter(
      "localization/output/motion_gate/max_yaw_rate_rad_s",
      this->output_motion_gate_max_yaw_rate_rad_s_);
  const auto nonnegative_or = [this](double value, double fallback,
                                     const char* name) {
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_WARN(this->get_logger(), "%s must be finite and >=0; using %.3f",
                  name, fallback);
      return fallback;
    }
    return value;
  };
  this->output_motion_gate_wheel_endpoint_max_age_s_ = nonnegative_or(
      this->output_motion_gate_wheel_endpoint_max_age_s_, 0.1,
      "motion_gate wheel_endpoint_max_age_s");
  this->output_motion_gate_distance_abs_tolerance_m_ = nonnegative_or(
      this->output_motion_gate_distance_abs_tolerance_m_, 0.75,
      "motion_gate distance_abs_tolerance_m");
  this->output_motion_gate_distance_rel_tolerance_ = nonnegative_or(
      this->output_motion_gate_distance_rel_tolerance_, 0.35,
      "motion_gate distance_rel_tolerance");
  this->output_motion_gate_reverse_tolerance_m_ = nonnegative_or(
      this->output_motion_gate_reverse_tolerance_m_, 0.1,
      "motion_gate reverse_tolerance_m");
  this->output_motion_gate_lateral_abs_tolerance_m_ = nonnegative_or(
      this->output_motion_gate_lateral_abs_tolerance_m_, 0.35,
      "motion_gate lateral_abs_tolerance_m");
  this->output_motion_gate_lateral_rel_tolerance_ = nonnegative_or(
      this->output_motion_gate_lateral_rel_tolerance_, 0.15,
      "motion_gate lateral_rel_tolerance");
  output_motion_gate_yaw_abs_tolerance_deg = nonnegative_or(
      output_motion_gate_yaw_abs_tolerance_deg, 5.0,
      "motion_gate yaw_abs_tolerance_deg");
  this->output_motion_gate_yaw_abs_tolerance_rad_ =
      output_motion_gate_yaw_abs_tolerance_deg * M_PI / 180.0;
  this->output_motion_gate_max_yaw_rate_rad_s_ = nonnegative_or(
      this->output_motion_gate_max_yaw_rate_rad_s_, 2.5,
      "motion_gate max_yaw_rate_rad_s");
  this->get_parameter("localization/use_odom_init", this->use_odom_init_);
  this->get_parameter("localization/initial_pose/use",
                      this->use_param_initial_pose_);
  this->get_parameter("localization/initial_pose/frame",
                      this->initial_pose_frame_);
  this->get_parameter("localization/initial_pose/x", this->initial_pose_x_);
  this->get_parameter("localization/initial_pose/y", this->initial_pose_y_);
  this->get_parameter("localization/initial_pose/z", this->initial_pose_z_);
  this->get_parameter("localization/initial_pose/roll",
                      this->initial_pose_roll_);
  this->get_parameter("localization/initial_pose/pitch",
                      this->initial_pose_pitch_);
  this->get_parameter("localization/initial_pose/yaw", this->initial_pose_yaw_);

  // Executor and GICP parameters. The scan pipeline has a dedicated worker;
  // this controls only the rclcpp callback executor. Zero keeps rclcpp's
  // hardware-concurrency default for unconstrained hosts.
  this->declare_parameter<int>("localization/executor_threads", 0);
  this->declare_parameter<int>("gicp/numThreads", 1);
  this->declare_parameter<int>("gicp/maxIterations", 32);
  // Each LM outer iteration may retry lambda with another full-cloud error
  // evaluation. Bound it independently so one difficult frame cannot hide
  // ten expensive error passes behind a small outer-iteration count.
  this->declare_parameter<int>("gicp/maxInnerIterations", 10);
  // Optional acquisition-only iteration limit. Zero reuses maxIterations.
  this->declare_parameter<int>("gicp/startupMaxIterations", 0);
  // Optional wall-clock budget for the iterative optimizer. Zero preserves
  // the unbounded historical behavior. At the cutoff the backend returns its
  // last fully linearized pose for ordinary LiDAR quality gates; the separate
  // hard result-age limit below fails closed on catastrophic overruns.
  this->declare_parameter<double>("gicp/maxOptimizationTimeMsCooperative", 0.0);
  // Stop starting new LM work this long before the cooperative deadline so a
  // final pose/Hessian/fitness evaluation can complete inside the frame.
  this->declare_parameter<double>("gicp/finalizationReserveMs", 0.0);
  // Optional wider budget before the first accepted GICP fix. This lets a
  // mid-run GNSS seed settle onto the map once without weakening the bounded
  // steady-state deadline. Zero reuses the normal budget.
  this->declare_parameter<double>(
      "gicp/startupMaxOptimizationTimeMsCooperative", 0.0);
  // Absolute fail-closed guard for any registration result that escapes the
  // cooperative budget because an individual library kernel is not
  // interruptible. This rejects the result; the cooperative deadline remains
  // the much tighter production compute bound.
  this->declare_parameter<double>("gicp/hardResultMaxAgeMs", 1000.0);
  this->declare_parameter<int>("gicp/startupAcceptedScans", 1);
  this->declare_parameter<double>("gicp/maxOptimizationTimeMs", -1.0);
  this->declare_parameter<int>("gicp/correspondenceRandomness", 20);
  this->declare_parameter<double>("gicp/maxCorrespondenceDistance", 1.0);
  this->declare_parameter<double>("gicp/transformationEpsilon", 0.0001);
  this->declare_parameter<double>("gicp/rotationEpsilon", 0.0001);
  // Optional deterministic cap after voxel filtering. Zero preserves every
  // voxel centroid; positive values bound dense-scene registration work.
  this->declare_parameter<int>("scan_max_points", 0);
  this->declare_parameter<bool>("scan_point_budget/spatial_balancing", false);
  this->declare_parameter<int>("scan_point_budget/azimuth_bins", 36);
  this->declare_parameter<int>("scan_point_budget/range_bins", 3);
  this->declare_parameter<double>("gicp/fitnessRejectThreshold", 1.0);
  this->declare_parameter<bool>("gicp/rejectLargeJumps", true);
  // Reject scans whose Hessian condition number proxy exceeds this threshold.
  // Straights typically run ~1e4; feature-poor corners spike to 1e8-1e9 and the
  // optimizer slides along the unconstrained axis. Set <=0 to disable the gate.
  // [P3 FIX 2026-07-10] Declared defaults aligned with the shipped yaml (the
  // calibrated values): a launch/namespace drift that missed the yaml key
  // previously activated a 5000x more aggressive trigger silently.
  this->declare_parameter<double>("gicp/hessianCondMax", 5.0e9);
  // Hessian rejection fires when condition number is high AND any of:
  //   - fitness exceeds the warn floor below
  //   - GICP applied a translation correction larger than transWarn
  //   - GICP applied a rotation correction larger than rotWarn
  // The latter two catch the optimizer sliding along an unconstrained axis: in
  // degenerate geometry IMU already provides a good prior, so a healthy GICP
  // correction is small. A large correction in degenerate geometry is the
  // slide signature even when fitness looks fine. Set any threshold <= 0 to
  // disable that specific OR branch.
  this->declare_parameter<double>("gicp/hessianFitnessWarnThreshold", 0.15);
  this->declare_parameter<double>("gicp/hessianTransWarnM", 1.0);
  this->declare_parameter<double>("gicp/hessianRotWarnDeg", 1.5);

  // Registration gating and degeneracy handling.
  // Rolling-median fitness baseline: absolute fitness thresholds calibrated on
  // a same-run map (floor 0.03-0.06) are meaningless on cross-run maps (floor
  // ~0.27), so gates operate on fitness / rolling-median instead.
  this->declare_parameter<bool>("gicp/fitnessBaseline/enable", true);
  this->declare_parameter<int>("gicp/fitnessBaseline/window", 201);
  this->declare_parameter<int>("gicp/fitnessBaseline/minSamples", 50);
  // Warm-up seed: expected per-map fitness floor used as the baseline until
  // minSamples accepted frames exist, so ratio gates / yaw veto are live from
  // frame 1 (0 = off; gates absolute-only during warm-up, pre-review behavior).
  this->declare_parameter<double>("gicp/fitnessBaseline/seedBaseline", 0.0);
  // Wrong-basin (bad-accept) gate: run-12 data shows good accepts at ratio
  // median 1.00 / p99 1.92, bad accepts (gt_err>20m) at median 1.34 / p90 2.55.
  this->declare_parameter<double>("gicp/fitnessRatioRejectThreshold", 2.0);
  // Degeneracy partial update: when the condition proxy crosses hessianCondMax,
  // project the correction instead of rejecting the whole scan (the old binary
  // reject produced 253-frame dead-reckoning streaks on run 12).
  this->declare_parameter<bool>("gicp/degeneracy/partialUpdate", true);
  // Full 6D coupled remapping (default) vs independent 3x3 rot/trans blocks.
  // full6d sees COUPLED rot/trans null directions (slide-along-a-wall = yaw +
  // lateral mix) that a blockwise analysis structurally cannot; rotation
  // coordinates are made commensurable with translation via couplingLengthM
  // (the typical constraint lever arm, ~point-cloud radius after cropping).
  this->declare_parameter<bool>("gicp/degeneracy/full6d", true);
  this->declare_parameter<double>("gicp/degeneracy/couplingLengthM", 20.0);
  this->declare_parameter<double>("gicp/degeneracy/relFloor6d", 0.02);
  this->declare_parameter<double>("gicp/degeneracy/relFloorRot", 0.02);
  this->declare_parameter<double>("gicp/degeneracy/relFloorTrans", 0.02);
  // Turn-aware yaw-consistency veto (Codex finding 1).
  this->declare_parameter<bool>("gicp/yawGate/enable", true);
  this->declare_parameter<double>("gicp/yawGate/maxCorrDeg", 1.5);
  this->declare_parameter<double>("gicp/yawGate/fitnessRatio", 1.2);
  // P1 yaw-safety: UNCONDITIONAL veto tier — fires regardless of fitness
  // ratio. The IMU prior's yaw cannot be this wrong over one scan gap on a
  // ground vehicle; any larger GICP yaw "correction" is a wrong basin with
  // plausible fitness (runs 19/20). <=0 disables the hard tier.
  this->declare_parameter<double>("gicp/rpGate/hardMaxCorrDeg", 3.0);
  this->declare_parameter<double>("gicp/yawGate/hardMaxCorrDeg", 8.0);
  // PR#6: bounds on the non-converged low-fitness fallback (see
  // performLocalization). Defaults from the PR validation replay.
  this->declare_parameter<double>("gicp/nonConvergedFitnessOkMaxTransM", 3.0);
  this->declare_parameter<double>("gicp/nonConvergedFitnessOkMaxRotDeg", 5.0);
  this->declare_parameter<int>("gicp/minCorrespondences", 500);
  this->declare_parameter<double>("gicp/minCorrespondenceRatio", 0.2);
  // ---- Algorithmic yaw-defect fixes (2026-07-05 deep-cause report) ----
  // Registration DoF for a ground vehicle. "4dof" (default) fixes roll+pitch
  // to the IMU prior inside the optimizer (translation+yaw free); "3dof"
  // additionally fixes yaw (translation-only registration); "planar" fixes
  // roll/pitch/z and solves exactly x/y/yaw. A warm-up value of zero fixes Z
  // from the first scan when initialization/static elevation supplies it;
  // "6dof" restores the unconstrained upstream behavior. Periodic 6-DoF
  // refresh is deliberately disabled in planar mode because it would reopen
  // the vertical drift path the mode is meant to close.
  this->declare_parameter<std::string>("gicp/dof/mode", "4dof");
  this->declare_parameter<int>("gicp/dof/full6dofEveryN", 10);
  this->declare_parameter<int>("gicp/dof/planarWarmupAcceptedScans", 1);
  // Soft attitude prior INSIDE the LM optimizer (info in rad^-2 on the
  // world-tangent residual vs the IMU-integrated initial guess). 0 = off.
  // Calibrate yawInfo against an offline Hessian analysis (start at ~0.2x
  // the observed median). rollPitchInfo only matters in 6dof mode.
  this->declare_parameter<double>("gicp/prior/yawInfo", 0.0);
  this->declare_parameter<double>("gicp/prior/rollPitchInfo", 0.0);
  // INS heading/pose prior (division of labor): IMU for propagation/deskew,
  // filtered_odom for the stable heading (+ optional position) prior.
  this->declare_parameter<bool>("localization/ins_prior/enable", true);
  this->declare_parameter<double>("localization/ins_prior/yaw_blend", 0.25);
  this->declare_parameter<double>("localization/ins_prior/max_yaw_step_deg",
                                  2.0);
  this->declare_parameter<double>("localization/ins_prior/sanity_max_yaw_deg",
                                  30.0);
  this->declare_parameter<double>("localization/ins_prior/pos_blend", 0.0);
  this->declare_parameter<double>(
      "localization/ins_prior/gicp_position_seed_blend", 0.0);
  this->declare_parameter<double>(
      "localization/ins_prior/gicp_position_seed_max_step_m", 20.0);
  this->declare_parameter<bool>("localization/ins_prior/require_rtk_fixed",
                                true);
  this->declare_parameter<double>("localization/ins_prior/max_yaw_sigma_deg",
                                  3.0);
  this->declare_parameter<bool>("localization/imu_heading_prior/enable", false);
  this->declare_parameter<double>("localization/imu_heading_prior/yaw_blend",
                                  1.0);
  this->declare_parameter<double>(
      "localization/imu_heading_prior/max_yaw_step_deg", 2.0);
  this->declare_parameter<double>(
      "localization/imu_heading_prior/sanity_max_yaw_deg", 30.0);
  this->declare_parameter<double>(
      "localization/imu_heading_prior/max_yaw_sigma_deg", 3.0);
  this->declare_parameter<double>(
      "localization/imu_heading_prior/max_time_error_s", 0.02);

  // Static race_common TTL candidate validation. This is independent of
  // gt_odom/VKS: no external live pose enters the objective or corrects x/y.
  this->declare_parameter<bool>("localization/track_constraint/enable", false);
  // Optional lap-time schedule. RDE telemetry supplies only a scalar
  // time-on-track index; the scheduled values tune the existing TTL safety
  // limits and never inject a transponder/GNSS pose into localization.
  this->declare_parameter<bool>("localization/track_tuning/enable", false);
  this->declare_parameter<std::string>(
      "localization/track_tuning/rde_telemetry_topic", "/rde/telemetry");
  this->declare_parameter<std::vector<double>>(
      "localization/track_tuning/time_breakpoints_s", std::vector<double>{});
  this->declare_parameter<std::vector<double>>(
      "localization/track_tuning/along_correction_gain",
      std::vector<double>{});
  this->declare_parameter<std::vector<double>>(
      "localization/track_tuning/max_along_correction_m",
      std::vector<double>{});
  this->declare_parameter<std::vector<double>>(
      "localization/track_tuning/max_applied_lateral_correction_m",
      std::vector<double>{});
  this->declare_parameter<std::string>(
      "localization/track_constraint/ttl_directory", "");
  this->declare_parameter<std::string>(
      "localization/track_constraint/line_name", "");
  this->declare_parameter<std::string>(
      "localization/track_constraint/trajectory_command_topic", "");
  this->declare_parameter<double>(
      "localization/track_constraint/corridor_margin_m", 0.75);
  this->declare_parameter<double>(
      "localization/track_constraint/prior_max_distance_m", 8.0);
  this->declare_parameter<double>(
      "localization/track_constraint/max_along_correction_m", 3.0);
  this->declare_parameter<double>(
      "localization/track_constraint/along_correction_gain", 1.0);
  this->declare_parameter<bool>(
      "localization/track_constraint/adaptive_along/enable", false);
  this->declare_parameter<double>(
      "localization/track_constraint/adaptive_along/"
      "low_information_per_correspondence",
      2.5);
  this->declare_parameter<double>(
      "localization/track_constraint/adaptive_along/"
      "full_information_per_correspondence",
      5.0);
  this->declare_parameter<double>(
      "localization/track_constraint/adaptive_along/min_gain", 0.0);
  this->declare_parameter<double>(
      "localization/track_constraint/recovery_along/max_raw_correction_m",
      0.0);
  this->declare_parameter<double>(
      "localization/track_constraint/recovery_along/gain", 0.0);
  this->declare_parameter<double>(
      "localization/track_constraint/recovery_along/"
      "max_applied_correction_m",
      0.0);
  this->declare_parameter<double>(
      "localization/track_constraint/max_lateral_correction_m", 3.0);
  this->declare_parameter<double>(
      "localization/track_constraint/max_centerline_distance_m", 0.0);
  this->declare_parameter<double>(
      "localization/track_constraint/max_applied_lateral_correction_m", 0.0);
  this->declare_parameter<bool>(
      "localization/track_constraint/adaptive_lateral/enable", false);
  this->declare_parameter<double>(
      "localization/track_constraint/adaptive_lateral/base_m", 0.5);
  this->declare_parameter<double>(
      "localization/track_constraint/adaptive_lateral/speed_distance_ratio",
      0.0);
  this->declare_parameter<double>(
      "localization/track_constraint/adaptive_lateral/max_m", 0.5);
  this->declare_parameter<double>(
      "localization/track_constraint/adaptive_lateral/max_scan_dt_s", 0.5);
  this->declare_parameter<double>(
      "localization/track_constraint/max_heading_error_deg", 60.0);
  this->declare_parameter<bool>(
      "localization/track_constraint/elevation/enable", true);
  this->declare_parameter<double>(
      "localization/track_constraint/elevation/max_distance_m", 8.0);
  this->declare_parameter<double>(
      "localization/track_constraint/elevation/min_span_m", 1.0);

  this->get_parameter("localization/executor_threads",
                      this->executor_threads_);
  this->get_parameter("gicp/numThreads", this->gicp_num_threads_);
  this->get_parameter("gicp/maxIterations", this->gicp_max_iter_);
  this->get_parameter("gicp/maxInnerIterations", this->gicp_max_inner_iter_);
  this->get_parameter("gicp/startupMaxIterations",
                      this->gicp_startup_max_iter_);
  this->get_parameter("gicp/maxOptimizationTimeMsCooperative",
                      this->gicp_max_optimization_time_ms_);
  this->get_parameter("gicp/finalizationReserveMs",
                      this->gicp_finalization_reserve_ms_);
  this->get_parameter("gicp/startupMaxOptimizationTimeMsCooperative",
                      this->gicp_startup_max_optimization_time_ms_);
  this->get_parameter("gicp/hardResultMaxAgeMs",
                      this->gicp_hard_result_max_age_ms_);
  this->get_parameter("gicp/startupAcceptedScans",
                      this->gicp_startup_accepted_scans_);
  if (!std::isfinite(this->gicp_startup_max_optimization_time_ms_) ||
      this->gicp_startup_max_optimization_time_ms_ < 0.0) {
    throw std::invalid_argument(
        "gicp/startupMaxOptimizationTimeMsCooperative must be finite and >= 0");
  }
  if (!std::isfinite(this->gicp_hard_result_max_age_ms_) ||
      this->gicp_hard_result_max_age_ms_ <= 0.0) {
    throw std::invalid_argument(
        "gicp/hardResultMaxAgeMs must be finite and > 0");
  }
  if (!std::isfinite(this->gicp_finalization_reserve_ms_) ||
      this->gicp_finalization_reserve_ms_ < 0.0) {
    throw std::invalid_argument(
        "gicp/finalizationReserveMs must be finite and >= 0");
  }
  if (this->gicp_startup_accepted_scans_ < 0) {
    throw std::invalid_argument("gicp/startupAcceptedScans must be >= 0");
  }
  if (this->gicp_startup_max_iter_ < 0) {
    throw std::invalid_argument("gicp/startupMaxIterations must be >= 0");
  }
  double legacy_max_optimization_time_ms = -1.0;
  this->get_parameter("gicp/maxOptimizationTimeMs",
                      legacy_max_optimization_time_ms);
  if (legacy_max_optimization_time_ms >= 0.0) {
    RCLCPP_WARN(
        this->get_logger(),
        "gicp/maxOptimizationTimeMs is deprecated; use "
        "gicp/maxOptimizationTimeMsCooperative. The budget is cooperative "
        "and includes source KD-tree/covariance preparation.");
    if (this->gicp_max_optimization_time_ms_ <= 0.0) {
      this->gicp_max_optimization_time_ms_ = legacy_max_optimization_time_ms;
    }
  }
  this->get_parameter("gicp/correspondenceRandomness",
                      this->gicp_corr_randomness_);
  this->get_parameter("gicp/maxCorrespondenceDistance",
                      this->gicp_max_corr_dist_);
  this->get_parameter("gicp/transformationEpsilon",
                      this->gicp_transformation_epsilon_);
  this->get_parameter("gicp/rotationEpsilon", this->gicp_rotation_epsilon_);
  int scan_max_points = 0;
  this->get_parameter("scan_max_points", scan_max_points);
  if (scan_max_points < 0) {
    throw std::invalid_argument("scan_max_points must be >= 0");
  }
  this->scan_max_points_ = static_cast<size_t>(scan_max_points);
  this->get_parameter("scan_point_budget/spatial_balancing",
                      this->scan_point_budget_spatial_balancing_);
  int scan_point_budget_azimuth_bins = 0;
  int scan_point_budget_range_bins = 0;
  this->get_parameter("scan_point_budget/azimuth_bins",
                      scan_point_budget_azimuth_bins);
  this->get_parameter("scan_point_budget/range_bins",
                      scan_point_budget_range_bins);
  if (scan_point_budget_azimuth_bins < 1 ||
      scan_point_budget_azimuth_bins > 360 ||
      scan_point_budget_range_bins < 1 || scan_point_budget_range_bins > 16) {
    throw std::invalid_argument(
        "scan point-budget bins must be within azimuth=[1,360], range=[1,16]");
  }
  this->scan_point_budget_azimuth_bins_ =
      static_cast<size_t>(scan_point_budget_azimuth_bins);
  this->scan_point_budget_range_bins_ =
      static_cast<size_t>(scan_point_budget_range_bins);
  this->get_parameter("gicp/fitnessRejectThreshold",
                      this->gicp_fitness_reject_threshold_);
  this->get_parameter("gicp/rejectLargeJumps", this->gicp_reject_large_jumps_);
  this->get_parameter("gicp/hessianCondMax", this->gicp_hessian_cond_max_);
  this->get_parameter("gicp/hessianFitnessWarnThreshold",
                      this->gicp_hessian_fitness_warn_);
  this->get_parameter("gicp/hessianTransWarnM",
                      this->gicp_hessian_trans_warn_m_);
  this->get_parameter("gicp/hessianRotWarnDeg",
                      this->gicp_hessian_rot_warn_deg_);
  this->get_parameter("gicp/fitnessBaseline/enable",
                      this->fitness_baseline_enable_);
  this->get_parameter("gicp/fitnessBaseline/window",
                      this->fitness_baseline_window_);
  this->get_parameter("gicp/fitnessBaseline/minSamples",
                      this->fitness_baseline_min_samples_);
  this->get_parameter("gicp/fitnessBaseline/seedBaseline",
                      this->fitness_baseline_seed_);
  this->get_parameter("gicp/fitnessRatioRejectThreshold",
                      this->fitness_ratio_reject_);
  this->get_parameter("gicp/degeneracy/partialUpdate",
                      this->degen_partial_update_enable_);
  this->get_parameter("gicp/degeneracy/full6d", this->degen_full6d_);
  this->get_parameter("gicp/degeneracy/couplingLengthM",
                      this->degen_coupling_length_m_);
  this->get_parameter("gicp/degeneracy/relFloor6d", this->degen_rel_floor_6d_);
  this->get_parameter("gicp/degeneracy/relFloorRot",
                      this->degen_rel_floor_rot_);
  this->get_parameter("gicp/degeneracy/relFloorTrans",
                      this->degen_rel_floor_trans_);
  this->get_parameter("gicp/yawGate/enable", this->yaw_gate_enable_);
  this->get_parameter("gicp/yawGate/maxCorrDeg", this->yaw_gate_max_corr_deg_);
  this->get_parameter("gicp/yawGate/fitnessRatio",
                      this->yaw_gate_fitness_ratio_);
  this->get_parameter("gicp/rpGate/hardMaxCorrDeg",
                      this->gicp_rp_hard_max_corr_deg_);
  this->get_parameter("gicp/yawGate/hardMaxCorrDeg",
                      this->yaw_gate_hard_max_corr_deg_);
  this->get_parameter("gicp/nonConvergedFitnessOkMaxTransM",
                      this->gicp_nonconv_ok_max_trans_m_);
  this->get_parameter("gicp/nonConvergedFitnessOkMaxRotDeg",
                      this->gicp_nonconv_ok_max_rot_deg_);
  this->get_parameter("gicp/minCorrespondences",
                      this->gicp_min_correspondences_);
  this->get_parameter("gicp/minCorrespondenceRatio",
                      this->gicp_min_corr_ratio_);
  this->get_parameter("gicp/dof/mode", this->gicp_dof_mode_);
  this->get_parameter("gicp/dof/full6dofEveryN", this->gicp_full6dof_every_n_);
  this->get_parameter("gicp/dof/planarWarmupAcceptedScans",
                      this->gicp_planar_warmup_accepted_scans_);
  this->get_parameter("gicp/prior/yawInfo", this->gicp_prior_yaw_info_);
  this->get_parameter("gicp/prior/rollPitchInfo",
                      this->gicp_prior_rollpitch_info_);
  this->get_parameter("localization/ins_prior/enable", this->ins_prior_enable_);
  this->get_parameter("localization/ins_prior/yaw_blend",
                      this->ins_prior_yaw_blend_);
  this->get_parameter("localization/ins_prior/max_yaw_step_deg",
                      this->ins_prior_max_yaw_step_deg_);
  this->get_parameter("localization/ins_prior/sanity_max_yaw_deg",
                      this->ins_prior_sanity_max_yaw_deg_);
  this->get_parameter("localization/ins_prior/pos_blend",
                      this->ins_prior_pos_blend_);
  this->get_parameter("localization/ins_prior/gicp_position_seed_blend",
                      this->ins_prior_gicp_position_seed_blend_);
  this->get_parameter("localization/ins_prior/gicp_position_seed_max_step_m",
                      this->ins_prior_gicp_position_seed_max_step_m_);
  this->get_parameter("localization/ins_prior/require_rtk_fixed",
                      this->ins_prior_require_rtk_);
  this->get_parameter("localization/ins_prior/max_yaw_sigma_deg",
                      this->ins_prior_max_yaw_sigma_deg_);
  this->get_parameter("localization/imu_heading_prior/enable",
                      this->imu_heading_prior_enable_);
  this->get_parameter("localization/imu_heading_prior/yaw_blend",
                      this->imu_heading_prior_yaw_blend_);
  this->get_parameter("localization/imu_heading_prior/max_yaw_step_deg",
                      this->imu_heading_prior_max_yaw_step_deg_);
  this->get_parameter("localization/imu_heading_prior/sanity_max_yaw_deg",
                      this->imu_heading_prior_sanity_max_yaw_deg_);
  this->get_parameter("localization/imu_heading_prior/max_yaw_sigma_deg",
                      this->imu_heading_prior_max_yaw_sigma_deg_);
  this->get_parameter("localization/imu_heading_prior/max_time_error_s",
                      this->imu_heading_prior_max_time_error_s_);
  TtlTrackConstraintConfig track_config;
  double track_heading_error_deg = 60.0;
  this->get_parameter("localization/track_constraint/enable",
                      this->track_constraint_enabled_);
  this->get_parameter("localization/track_constraint/ttl_directory",
                      this->track_ttl_directory_);
  this->get_parameter("localization/track_constraint/line_name",
                      track_config.line_name);
  this->get_parameter("localization/track_constraint/trajectory_command_topic",
                      this->track_trajectory_command_topic_);
  track_config.require_active_line =
      !this->track_trajectory_command_topic_.empty();
  this->get_parameter("localization/track_constraint/corridor_margin_m",
                      track_config.corridor_margin_m);
  this->get_parameter("localization/track_constraint/prior_max_distance_m",
                      track_config.prior_max_distance_m);
  this->get_parameter("localization/track_constraint/max_along_correction_m",
                      track_config.max_along_correction_m);
  this->get_parameter("localization/track_constraint/along_correction_gain",
                      track_config.along_correction_gain);
  this->track_along_correction_gain_ = track_config.along_correction_gain;
  this->track_max_along_correction_m_ = track_config.max_along_correction_m;
  this->get_parameter("localization/track_tuning/enable",
                      this->track_time_tuning_enabled_);
  this->get_parameter("localization/track_tuning/rde_telemetry_topic",
                      this->track_time_telemetry_topic_);
  this->get_parameter("localization/track_tuning/time_breakpoints_s",
                      this->track_time_breakpoints_s_);
  this->get_parameter("localization/track_tuning/along_correction_gain",
                      this->track_time_along_gains_);
  this->get_parameter("localization/track_tuning/max_along_correction_m",
                      this->track_time_max_along_corrections_m_);
  this->get_parameter(
      "localization/track_tuning/max_applied_lateral_correction_m",
      this->track_time_max_applied_lateral_m_);
  this->get_parameter("localization/track_constraint/adaptive_along/enable",
                      this->track_adaptive_along_enabled_);
  this->get_parameter("localization/track_constraint/adaptive_along/"
                      "low_information_per_correspondence",
                      this->track_adaptive_along_low_info_per_corr_);
  this->get_parameter("localization/track_constraint/adaptive_along/"
                      "full_information_per_correspondence",
                      this->track_adaptive_along_full_info_per_corr_);
  this->get_parameter("localization/track_constraint/adaptive_along/min_gain",
                      this->track_adaptive_along_min_gain_);
  this->get_parameter(
      "localization/track_constraint/recovery_along/max_raw_correction_m",
      this->track_recovery_along_max_raw_correction_m_);
  this->get_parameter(
      "localization/track_constraint/recovery_along/gain",
      this->track_recovery_along_gain_);
  this->get_parameter(
      "localization/track_constraint/recovery_along/"
      "max_applied_correction_m",
      this->track_recovery_along_max_applied_correction_m_);
  this->get_parameter("localization/track_constraint/max_lateral_correction_m",
                      track_config.max_lateral_correction_m);
  this->get_parameter("localization/track_constraint/max_centerline_distance_m",
                      track_config.max_centerline_distance_m);
  this->get_parameter(
      "localization/track_constraint/max_applied_lateral_correction_m",
      track_config.max_applied_lateral_correction_m);
  this->track_lateral_fallback_limit_m_ =
      track_config.max_applied_lateral_correction_m;
  this->get_parameter("localization/track_constraint/adaptive_lateral/enable",
                      this->track_adaptive_lateral_enabled_);
  this->get_parameter("localization/track_constraint/adaptive_lateral/base_m",
                      this->track_adaptive_lateral_base_m_);
  this->get_parameter(
      "localization/track_constraint/adaptive_lateral/speed_distance_ratio",
      this->track_adaptive_lateral_speed_distance_ratio_);
  this->get_parameter("localization/track_constraint/adaptive_lateral/max_m",
                      this->track_adaptive_lateral_max_m_);
  this->get_parameter(
      "localization/track_constraint/adaptive_lateral/max_scan_dt_s",
      this->track_adaptive_lateral_max_scan_dt_s_);
  this->get_parameter("localization/track_constraint/max_heading_error_deg",
                      track_heading_error_deg);
  this->get_parameter("localization/track_constraint/elevation/enable",
                      this->track_elevation_enabled_);
  this->get_parameter("localization/track_constraint/elevation/max_distance_m",
                      track_config.elevation_max_distance_m);
  this->get_parameter("localization/track_constraint/elevation/min_span_m",
                      track_config.elevation_min_span_m);
  track_config.max_heading_error_rad = track_heading_error_deg * M_PI / 180.0;
  if (this->executor_threads_ < 0) {
    throw std::invalid_argument("localization/executor_threads must be >= 0");
  }
  if (this->gicp_num_threads_ < 1) {
    throw std::invalid_argument("gicp/numThreads must be >= 1");
  }
  {
    auto sanitize = [this](const char *name, double &v, double lo, double hi,
                           double fallback) {
      if (!std::isfinite(v) || v < lo || v > hi) {
        RCLCPP_WARN(this->get_logger(),
                    "localization/imu_heading_prior/%s = %.3f outside "
                    "[%.3f, %.3f]; using %.3f",
                    name, v, lo, hi, fallback);
        v = fallback;
      }
    };
    sanitize("yaw_blend", this->imu_heading_prior_yaw_blend_, 0.0, 1.0, 1.0);
    sanitize("max_yaw_step_deg", this->imu_heading_prior_max_yaw_step_deg_, 0.0,
             90.0, 2.0);
    sanitize("sanity_max_yaw_deg", this->imu_heading_prior_sanity_max_yaw_deg_,
             1e-3, 180.0, 30.0);
    sanitize("max_time_error_s", this->imu_heading_prior_max_time_error_s_, 0.0,
             1.0, 0.02);
  }
  RCLCPP_INFO(this->get_logger(),
              "IMU heading prior: %s (yaw_blend=%.2f, max_step=%.1fdeg, "
              "sanity=%.1fdeg, max_yaw_sigma=%.1fdeg, max_time_error=%.3fs; "
              "position never consumed)",
              this->imu_heading_prior_enable_ ? "ENABLED" : "disabled",
              this->imu_heading_prior_yaw_blend_,
              this->imu_heading_prior_max_yaw_step_deg_,
              this->imu_heading_prior_sanity_max_yaw_deg_,
              this->imu_heading_prior_max_yaw_sigma_deg_,
              this->imu_heading_prior_max_time_error_s_);
  if (this->imu_heading_prior_enable_ && this->ins_prior_enable_) {
    RCLCPP_WARN(this->get_logger(),
                "Both IMU heading prior and filtered-odom INS prior are "
                "enabled; IMU heading takes precedence and the sources will "
                "not be stacked");
  }
  if (this->gicp_max_inner_iter_ < 1) {
    throw std::invalid_argument("gicp/maxInnerIterations must be >= 1");
  }
  // [REVIEW FIX 2026-07-08 P3] Sanitize: defaults are safe, but bad YAML values
  // would invert the prior's semantics silently — a negative max_yaw_step_deg
  // reaches std::clamp with REVERSED bounds (UB), a negative yaw_blend steers
  // AWAY from the INS, and a negative sanity threshold always trips (silently
  // disabling the prior). max_yaw_sigma_deg <= 0 is a documented gate-disable
  // and is left as-is.
  {
    auto sanitize = [this](const char *name, double &v, double lo, double hi,
                           double fallback) {
      if (!std::isfinite(v) || v < lo || v > hi) {
        RCLCPP_WARN(
            this->get_logger(),
            "localization/ins_prior/%s = %.3f outside [%.3f, %.3f]; using %.3f",
            name, v, lo, hi, fallback);
        v = fallback;
      }
    };
    sanitize("yaw_blend", this->ins_prior_yaw_blend_, 0.0, 1.0, 0.25);
    sanitize("max_yaw_step_deg", this->ins_prior_max_yaw_step_deg_, 0.0, 90.0,
             2.0);
    sanitize("sanity_max_yaw_deg", this->ins_prior_sanity_max_yaw_deg_, 1e-3,
             180.0, 30.0);
    sanitize("pos_blend", this->ins_prior_pos_blend_, 0.0, 1.0, 0.0);
    sanitize("gicp_position_seed_blend",
             this->ins_prior_gicp_position_seed_blend_, 0.0, 1.0, 0.0);
    sanitize("gicp_position_seed_max_step_m",
             this->ins_prior_gicp_position_seed_max_step_m_, 0.0, 1000.0, 20.0);
  }
  RCLCPP_INFO(
      this->get_logger(),
      "INS prior: %s (yaw_blend=%.2f, max_step=%.1fdeg, sanity=%.1fdeg, "
      "pos_blend=%.2f, gicp_pos_seed=[blend=%.2f,max=%.1fm], "
      "rtk_only=%s, max_yaw_sigma=%.1fdeg) — "
      "configured IMU subscription for propagation/deskew, filtered_odom for "
      "stable heading",
      this->ins_prior_enable_ ? "ENABLED" : "disabled",
      this->ins_prior_yaw_blend_, this->ins_prior_max_yaw_step_deg_,
      this->ins_prior_sanity_max_yaw_deg_, this->ins_prior_pos_blend_,
      this->ins_prior_gicp_position_seed_blend_,
      this->ins_prior_gicp_position_seed_max_step_m_,
      this->ins_prior_require_rtk_ ? "yes" : "no",
      this->ins_prior_max_yaw_sigma_deg_);
  if (this->gicp_dof_mode_ != "6dof" && this->gicp_dof_mode_ != "4dof" &&
      this->gicp_dof_mode_ != "3dof" && this->gicp_dof_mode_ != "planar") {
    RCLCPP_WARN(this->get_logger(),
                "gicp/dof/mode '%s' unknown; falling back to 6dof",
                this->gicp_dof_mode_.c_str());
    this->gicp_dof_mode_ = "6dof";
  }
  if (this->gicp_planar_warmup_accepted_scans_ < 0) {
    throw std::invalid_argument(
        "gicp/dof/planarWarmupAcceptedScans must be >= 0");
  }
  RCLCPP_INFO(this->get_logger(),
              "GICP DoF: %s (full 6dof every %d scans; planar warm-up=%d "
              "accepted scans), "
              "rotation prior info yaw=%.1f rp=%.1f rad^-2",
              this->gicp_dof_mode_.c_str(), this->gicp_full6dof_every_n_,
              this->gicp_planar_warmup_accepted_scans_,
              this->gicp_prior_yaw_info_, this->gicp_prior_rollpitch_info_);
  const auto require_positive_finite = [](const char *name, double value,
                                          bool allow_zero = false) {
    if (!std::isfinite(value) || (allow_zero ? value < 0.0 : value <= 0.0)) {
      throw std::invalid_argument(std::string(name) +
                                  (allow_zero ? " must be finite and >= 0"
                                              : " must be finite and > 0"));
    }
  };
  require_positive_finite("track corridor_margin_m",
                          track_config.corridor_margin_m, true);
  require_positive_finite("track prior_max_distance_m",
                          track_config.prior_max_distance_m);
  require_positive_finite("track max_along_correction_m",
                          track_config.max_along_correction_m);
  if (!std::isfinite(track_config.along_correction_gain) ||
      track_config.along_correction_gain < 0.0 ||
      track_config.along_correction_gain > 1.0) {
    throw std::invalid_argument(
        "track along_correction_gain must be within [0, 1]");
  }
  if (!std::isfinite(this->track_adaptive_along_low_info_per_corr_) ||
      !std::isfinite(this->track_adaptive_along_full_info_per_corr_) ||
      this->track_adaptive_along_low_info_per_corr_ < 0.0 ||
      this->track_adaptive_along_full_info_per_corr_ <=
          this->track_adaptive_along_low_info_per_corr_) {
    throw std::invalid_argument(
        "adaptive along information thresholds must satisfy 0 <= low < full");
  }
  if (!std::isfinite(this->track_adaptive_along_min_gain_) ||
      this->track_adaptive_along_min_gain_ < 0.0 ||
      this->track_adaptive_along_min_gain_ >
          this->track_along_correction_gain_) {
    throw std::invalid_argument(
        "adaptive along min_gain must be within [0, along_correction_gain]");
  }
  require_positive_finite("track recovery_along/max_raw_correction_m",
                          this->track_recovery_along_max_raw_correction_m_,
                          true);
  if (!std::isfinite(this->track_recovery_along_gain_) ||
      this->track_recovery_along_gain_ < 0.0 ||
      this->track_recovery_along_gain_ >
          this->track_along_correction_gain_) {
    throw std::invalid_argument(
        "track recovery_along/gain must be within "
        "[0, along_correction_gain]");
  }
  require_positive_finite(
      "track recovery_along/max_applied_correction_m",
      this->track_recovery_along_max_applied_correction_m_, true);
  const bool recovery_along_partially_configured =
      this->track_recovery_along_max_raw_correction_m_ > 0.0 ||
      this->track_recovery_along_gain_ > 0.0 ||
      this->track_recovery_along_max_applied_correction_m_ > 0.0;
  if (recovery_along_partially_configured &&
      (this->track_recovery_along_max_raw_correction_m_ <=
           track_config.max_along_correction_m ||
       this->track_recovery_along_gain_ <= 0.0 ||
       this->track_recovery_along_max_applied_correction_m_ <= 0.0 ||
       this->track_recovery_along_max_applied_correction_m_ >
           this->track_recovery_along_max_raw_correction_m_)) {
    throw std::invalid_argument(
        "track recovery_along requires max_raw_correction_m > the normal "
        "max_along_correction_m, gain > 0, and 0 < "
        "max_applied_correction_m <= max_raw_correction_m");
  }
  require_positive_finite("track max_lateral_correction_m",
                          track_config.max_lateral_correction_m);
  require_positive_finite("track max_centerline_distance_m",
                          track_config.max_centerline_distance_m, true);
  require_positive_finite("track max_applied_lateral_correction_m",
                          track_config.max_applied_lateral_correction_m, true);
  require_positive_finite("track adaptive_lateral/base_m",
                          this->track_adaptive_lateral_base_m_);
  require_positive_finite("track adaptive_lateral/speed_distance_ratio",
                          this->track_adaptive_lateral_speed_distance_ratio_,
                          true);
  require_positive_finite("track adaptive_lateral/max_m",
                          this->track_adaptive_lateral_max_m_);
  require_positive_finite("track adaptive_lateral/max_scan_dt_s",
                          this->track_adaptive_lateral_max_scan_dt_s_);
  if (this->track_adaptive_lateral_max_m_ <
      this->track_adaptive_lateral_base_m_) {
    throw std::invalid_argument(
        "track adaptive_lateral/max_m must be >= base_m");
  }
  if (this->track_adaptive_lateral_max_m_ >
      track_config.max_lateral_correction_m) {
    throw std::invalid_argument(
        "track adaptive_lateral/max_m must not exceed the hard lateral "
        "correction limit");
  }
  require_positive_finite("track max_heading_error_deg",
                          track_heading_error_deg);
  require_positive_finite("track elevation/max_distance_m",
                          track_config.elevation_max_distance_m);
  require_positive_finite("track elevation/min_span_m",
                          track_config.elevation_min_span_m, true);
  if (track_heading_error_deg > 180.0) {
    throw std::invalid_argument("track max_heading_error_deg must be <= 180");
  }
  if (!this->track_time_breakpoints_s_.empty()) {
    for (size_t i = 0; i < this->track_time_breakpoints_s_.size(); ++i) {
      const double breakpoint = this->track_time_breakpoints_s_[i];
      if (!std::isfinite(breakpoint) || breakpoint < 0.0 ||
          (i > 0 && breakpoint <= this->track_time_breakpoints_s_[i - 1])) {
        throw std::invalid_argument(
            "track_tuning/time_breakpoints_s must be finite, nonnegative, "
            "and strictly increasing");
      }
    }
  }
  const auto validate_track_schedule = [this](const char *name,
                                                const std::vector<double> &values,
                                                bool allow_zero,
                                                bool unit_interval) {
    if (!values.empty() &&
        values.size() != this->track_time_breakpoints_s_.size()) {
      throw std::invalid_argument(std::string(name) +
                                  " must have the same length as "
                                  "track_tuning/time_breakpoints_s");
    }
    for (const double value : values) {
      if (!std::isfinite(value) ||
          (unit_interval ? (value < 0.0 || value > 1.0)
                         : (allow_zero ? value < 0.0 : value <= 0.0))) {
        throw std::invalid_argument(std::string(name) +
                                    " contains an invalid value");
      }
    }
  };
  validate_track_schedule("track_tuning/along_correction_gain",
                          this->track_time_along_gains_, true, true);
  validate_track_schedule("track_tuning/max_along_correction_m",
                          this->track_time_max_along_corrections_m_, false,
                          false);
  validate_track_schedule(
      "track_tuning/max_applied_lateral_correction_m",
      this->track_time_max_applied_lateral_m_, false, false);
  for (const double value : this->track_time_max_along_corrections_m_) {
    if (value > track_config.max_along_correction_m) {
      throw std::invalid_argument(
          "track_tuning/max_along_correction_m cannot exceed "
          "track_constraint/max_along_correction_m");
    }
  }
  if (this->track_time_tuning_enabled_ &&
      !this->track_constraint_enabled_) {
    RCLCPP_WARN(this->get_logger(),
                "track_tuning/enable=true but track_constraint/enable=false; "
                "the schedule will remain inactive");
  }
  if (this->track_time_tuning_enabled_ &&
      this->track_time_breakpoints_s_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "track_tuning/enable=true but no time schedule is configured; "
                "base TTL limits will be used");
  }
  RCLCPP_INFO(
      this->get_logger(),
      "Track-time tuning: %s (telemetry='%s', %zu breakpoints; values are "
      "TTL limits only)",
      this->track_time_tuning_enabled_ ? "ENABLED" : "disabled",
      this->track_time_telemetry_topic_.c_str(),
      this->track_time_breakpoints_s_.size());
  if (this->track_constraint_enabled_) {
    if (this->track_ttl_directory_.empty()) {
      throw std::invalid_argument("track_constraint/ttl_directory is required "
                                  "when the gate is enabled");
    }
    if (!this->track_trajectory_command_topic_.empty() &&
        !track_config.line_name.empty()) {
      throw std::invalid_argument(
          "track_constraint line_name and trajectory_command_topic are "
          "mutually exclusive");
    }
    this->track_constraint_ =
        std::make_unique<TtlTrackConstraint>(track_config);
    std::string track_error;
    if (!this->track_constraint_->loadDirectory(this->track_ttl_directory_,
                                                &track_error)) {
      throw std::runtime_error("Failed to load race_common TTLs: " +
                               track_error);
    }
    if (this->track_elevation_enabled_ &&
        this->track_constraint_->elevationLineCount() == 0) {
      throw std::runtime_error(
          "TTL elevation enabled but no line has a valid elevation profile");
    }
    RCLCPP_INFO(
        this->get_logger(),
        "TTL track constraint ENABLED: directory='%s', line='%s', "
        "command_topic='%s', lines=%zu, "
        "z_lines=%zu, "
        "corridor_margin=%.2fm, correction=[along<=%.2fm,gain=%.2f,"
        "lateral_reject<=%.2fm,lateral_apply<=%.2fm,centerline<=%.2fm], "
        "recovery_along=[raw<=%.2fm,gain=%.2f,apply<=%.2fm], "
        "adaptive_lateral=[%s,base=%.2fm,ratio=%.3f,max=%.2fm,dt<=%.2fs], "
        "elevation=%s",
        this->track_ttl_directory_.c_str(),
        track_config.line_name.empty() ? "<all>"
                                       : track_config.line_name.c_str(),
        this->track_trajectory_command_topic_.empty()
            ? "<static>"
            : this->track_trajectory_command_topic_.c_str(),
        this->track_constraint_->lineCount(),
        this->track_constraint_->elevationLineCount(),
        track_config.corridor_margin_m, track_config.max_along_correction_m,
        track_config.along_correction_gain,
        track_config.max_lateral_correction_m,
        track_config.max_applied_lateral_correction_m,
        track_config.max_centerline_distance_m,
        this->track_recovery_along_max_raw_correction_m_,
        this->track_recovery_along_gain_,
        this->track_recovery_along_max_applied_correction_m_,
        this->track_adaptive_lateral_enabled_ ? "ON" : "off",
        this->track_adaptive_lateral_base_m_,
        this->track_adaptive_lateral_speed_distance_ratio_,
        this->track_adaptive_lateral_max_m_,
        this->track_adaptive_lateral_max_scan_dt_s_,
        this->track_elevation_enabled_ ? "ON" : "off");
  } else {
    this->track_elevation_enabled_ = false;
  }
  if (this->fitness_baseline_window_ < 3)
    this->fitness_baseline_window_ = 3;
  if (this->fitness_baseline_min_samples_ < 3)
    this->fitness_baseline_min_samples_ = 3;
  RCLCPP_INFO(
      this->get_logger(),
      "Registration gating: fitness baseline %s (window=%d, min=%d), ratio_reject=%.2f, "
      "partial_update=%s (%s, L=%.1fm, floor6d=%.3f, block floors rot=%.3f "
      "trans=%.3f), "
      "yaw_gate=%s (max=%.2fdeg, ratio>%.2f)",
      this->fitness_baseline_enable_ ? "ON" : "OFF",
      this->fitness_baseline_window_, this->fitness_baseline_min_samples_,
      this->fitness_ratio_reject_,
      this->degen_partial_update_enable_ ? "ON" : "OFF",
      this->degen_full6d_ ? "full6d" : "blockwise",
      this->degen_coupling_length_m_, this->degen_rel_floor_6d_,
      this->degen_rel_floor_rot_, this->degen_rel_floor_trans_,
      this->yaw_gate_enable_ ? "ON" : "OFF", this->yaw_gate_max_corr_deg_,
      this->yaw_gate_fitness_ratio_);

  // Preprocessing parameters
  this->declare_parameter<double>("scan_crop_size", 80.0);
  this->declare_parameter<double>("scan_min_range", 0.0);
  this->declare_parameter<double>("scan_max_range", 0.0);
  this->declare_parameter<bool>("scan_downsample_enabled", true);
  this->declare_parameter<double>("scan_downsample", 0.3);

  this->get_parameter("scan_crop_size", this->crop_size_);
  this->get_parameter("scan_min_range", this->scan_min_range_);
  this->get_parameter("scan_max_range", this->scan_max_range_);
  this->get_parameter("scan_downsample_enabled", this->vf_use_);
  this->get_parameter("scan_downsample", this->vf_res_);
  if (this->scan_min_range_ < 0.0 || this->scan_max_range_ < 0.0 ||
      (this->scan_max_range_ > 0.0 &&
       this->scan_max_range_ <= this->scan_min_range_)) {
    throw std::invalid_argument(
        "scan range must satisfy 0 <= scan_min_range < scan_max_range, "
        "or scan_max_range=0 to disable the upper bound");
  }

  // IMU and deskewing parameters
  this->declare_parameter<bool>("enable_deskew", true);
  this->declare_parameter<double>("gravity", 9.81);
  this->declare_parameter<int>("imu_buffer_size", 2000);
  this->declare_parameter<double>(
      "localization/deskew/future_imu_wait_timeout_s", 1.0);
  this->declare_parameter<double>(
      "localization/deskew/trajectory_knot_interval_s", 0.002);

  this->get_parameter("enable_deskew", this->deskew_);
  this->get_parameter("gravity", this->gravity_);
  this->get_parameter("imu_buffer_size", this->imu_buffer_size_);
  this->get_parameter("localization/deskew/future_imu_wait_timeout_s",
                      this->future_imu_wait_timeout_s_);
  this->get_parameter("localization/deskew/trajectory_knot_interval_s",
                      this->deskew_knot_interval_s_);
  if (this->future_imu_wait_timeout_s_ < 0.0) {
    throw std::invalid_argument(
        "localization/deskew/future_imu_wait_timeout_s must be >= 0");
  }
  if (!std::isfinite(this->deskew_knot_interval_s_) ||
      this->deskew_knot_interval_s_ <= 0.0 ||
      this->deskew_knot_interval_s_ > 0.02) {
    throw std::invalid_argument(
        "localization/deskew/trajectory_knot_interval_s must be in (0, 0.02]");
  }

  this->declare_parameter<bool>("localization/flip_y", false);
  this->get_parameter("localization/flip_y", this->flip_y_);

  // Multi-LiDAR concatenation: merge nearest-in-time aux scans into the primary
  // PointCloud2 before the existing pipeline runs. Aux XYZ are transformed into
  // the primary sensor frame via TF (URDF), and per-point timestamps are
  // rebased by the inter-header dt so the merged sweep shares one clock.
  this->declare_parameter<bool>("localization/lidar_concat/enabled", false);
  this->declare_parameter<std::vector<std::string>>(
      "localization/lidar_concat/aux_topics", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>(
      "localization/lidar_concat/aux_frames", std::vector<std::string>{});
  this->declare_parameter<double>("localization/lidar_concat/time_threshold",
                                  0.05);
  // P4#3: 200 for parity with GLIM's lidar_concat. At 10 Hz a depth of 20 is
  // only 2 s of aux history — a brief aux-stream stall or replay burst drops
  // the matching scan and the frame silently degrades to fewer LiDARs (run-12
  // throttled logs: only ~35% of sampled scans merged 2/2). 200 = ~20 s.
  this->declare_parameter<int>("localization/lidar_concat/buffer_size", 200);
  // Measured residual per-aux point-clock correction (seconds, ADDED to both
  // header and absolute point times). A header phase delta alone is NOT
  // evidence of clock error when every Iris is PTP-synchronized (runs 19/20's
  // "80-90 ms" were header-delta extrema; the geometric point-time regression
  // measured |offset| < 11 ms). Keep zero until a geometric point-time
  // measurement establishes a different value. Order matches aux_topics;
  // missing entries = 0.
  this->declare_parameter<std::vector<double>>(
      "localization/lidar_concat/aux_time_offsets", std::vector<double>{});
  // Mirror GLIM's explicit driver contract. FLOAT64 normally carries
  // scan-relative seconds (Laguna); only reinterpret its raw bytes as uint64
  // epoch nanoseconds when this is true. UINT8[8] remains an absolute carrier
  // regardless of this setting.
  this->declare_parameter<bool>(
      "localization/lidar_concat/float64_time_is_epoch_ns", false);
  this->declare_parameter<bool>(
      "localization/lidar_concat/float64_time_fail_on_mismatch", true);
  // Luminar acceptance gate: absolute point-time endpoint-range error
  // max(|min-min|,|max-max|) <= this. Header time is only a tie-break; the
  // 0.1 s header threshold stays solely for non-Luminar fallback matching.
  this->declare_parameter<double>(
      "localization/lidar_concat/luminar_point_time_threshold_s", 0.010);
  // Arrival-time (steady clock) deadline for a pending front cloud in the
  // async synchronizer. Bounds live latency when an aux is lost/stalled; it is
  // not a point-clock correction and never alters timestamps.
  this->declare_parameter<double>(
      "localization/lidar_concat/future_aux_wait_timeout_s", 0.150);
  // Pending-front queue depth that triggers a queue-pressure release of the
  // OLDEST front (with whatever matched). A capacity bound, never a drop.
  this->declare_parameter<int>("localization/lidar_concat/primary_queue_size",
                               8);
  // Timestamp-based GICP admission cap. Zero keeps every valid primary scan;
  // IMU callbacks and state propagation are never decimated.
  this->declare_parameter<double>("localization/registration_rate_hz", 0.0);
  // Offline aux-extrinsic resolution (mirrors GLIM; no live TF needed).
  this->declare_parameter<std::string>(
      "localization/lidar_concat/primary_frame", "luminar_front");
  this->declare_parameter<std::string>("localization/lidar_concat/urdf_path",
                                       "");
  this->declare_parameter<std::vector<double>>(
      "localization/lidar_concat/aux_static_transforms", std::vector<double>{});
  // Strict merge guard. When require_all_aux=true, a scan that fails to merge
  // every configured aux is NOT localized on fewer LiDARs -- it is skipped.
  // Brief transient misses (buffers warming up, a dropped aux frame) are
  // tolerated up to max_consecutive_aux_merge_failures; past that, the node
  // aborts if abort_on_merge_failure=true, otherwise it keeps skipping
  // non-fatally (louder warning). require_all_aux=false localizes on whatever
  // aux merged (no skip/abort).
  this->declare_parameter<bool>("localization/lidar_concat/require_all_aux",
                                false);
  this->declare_parameter<bool>(
      "localization/lidar_concat/abort_on_merge_failure", true);
  this->declare_parameter<int>(
      "localization/lidar_concat/max_consecutive_aux_merge_failures", 10);

  this->get_parameter("localization/lidar_concat/enabled",
                      this->concat_enabled_);
  std::vector<std::string> aux_topics_param, aux_frames_param;
  this->get_parameter("localization/lidar_concat/aux_topics", aux_topics_param);
  this->get_parameter("localization/lidar_concat/aux_frames", aux_frames_param);
  this->get_parameter("localization/lidar_concat/time_threshold",
                      this->concat_time_threshold_);
  // [P3 FIX 2026-07-10] Negative threshold silently drops every aux merge.
  if (!std::isfinite(this->concat_time_threshold_) ||
      this->concat_time_threshold_ < 0.0) {
    RCLCPP_WARN(
        this->get_logger(),
        "localization/lidar_concat/time_threshold=%.3f invalid; using 0.05",
        this->concat_time_threshold_);
    this->concat_time_threshold_ = 0.05;
  }
  int concat_buffer_size_int = 20;
  this->get_parameter("localization/lidar_concat/buffer_size",
                      concat_buffer_size_int);
  this->get_parameter("localization/lidar_concat/aux_time_offsets",
                      this->concat_aux_time_offsets_);
  this->get_parameter("localization/lidar_concat/float64_time_is_epoch_ns",
                      this->concat_float64_time_is_epoch_ns_);
  this->get_parameter("localization/lidar_concat/float64_time_fail_on_mismatch",
                      this->concat_float64_time_fail_on_mismatch_);
  // Fail LOUD on invalid offsets (GLIM config-loader policy): a NaN/inf or
  // extreme value would flow into point-range matching and the int64 ns
  // conversion in shiftCloudTimestamps (UB / corrupted absolute timestamps).
  // Real residual point-clock offsets are < 11 ms; 0.5 s is a generous bound
  // that still catches header-phase values pasted in by mistake being
  // combined with unit errors (e.g. ms entered as s stays within the bound,
  // but garbage like 1e9 or NaN cannot start the node).
  for (size_t i = 0; i < this->concat_aux_time_offsets_.size(); ++i) {
    const double v = this->concat_aux_time_offsets_[i];
    if (!std::isfinite(v) || std::abs(v) > 0.5) {
      RCLCPP_FATAL(
          this->get_logger(),
          "localization/lidar_concat/aux_time_offsets[%zu] = %g is invalid "
          "(must be finite and |v| <= 0.5 s); refusing to start",
          i, v);
      throw std::runtime_error("lidar_concat: invalid aux_time_offsets entry");
    }
  }
  this->get_parameter(
      "localization/lidar_concat/luminar_point_time_threshold_s",
      this->concat_luminar_point_threshold_);
  this->get_parameter("localization/lidar_concat/future_aux_wait_timeout_s",
                      this->concat_future_aux_wait_s_);
  int primary_queue_size_int = 8;
  this->get_parameter("localization/lidar_concat/primary_queue_size",
                      primary_queue_size_int);
  this->get_parameter("localization/registration_rate_hz",
                      this->registration_rate_hz_);
  if (!std::isfinite(this->registration_rate_hz_) ||
      this->registration_rate_hz_ < 0.0) {
    throw std::invalid_argument(
        "localization/registration_rate_hz must be finite and >= 0");
  }
  this->registration_rate_limiter_.configure(this->registration_rate_hz_);
  if (!std::isfinite(this->concat_luminar_point_threshold_) ||
      this->concat_luminar_point_threshold_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "localization/lidar_concat/luminar_point_time_threshold_s=%.3f "
                "invalid; using 0.010",
                this->concat_luminar_point_threshold_);
    this->concat_luminar_point_threshold_ = 0.010;
  }
  if (!std::isfinite(this->concat_future_aux_wait_s_) ||
      this->concat_future_aux_wait_s_ < 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "localization/lidar_concat/future_aux_wait_timeout_s=%.3f "
                "invalid; using 0.150",
                this->concat_future_aux_wait_s_);
    this->concat_future_aux_wait_s_ = 0.150;
  }
  this->concat_primary_queue_size_ =
      static_cast<size_t>(std::max(1, primary_queue_size_int));
  this->concat_buffer_size_ =
      static_cast<size_t>(std::max(1, concat_buffer_size_int));
  this->get_parameter("localization/lidar_concat/primary_frame",
                      this->concat_primary_frame_);
  this->get_parameter("localization/lidar_concat/urdf_path",
                      this->concat_urdf_path_);
  this->get_parameter("localization/lidar_concat/require_all_aux",
                      this->concat_require_all_aux_);
  this->get_parameter("localization/lidar_concat/abort_on_merge_failure",
                      this->concat_abort_on_merge_failure_);
  this->get_parameter(
      "localization/lidar_concat/max_consecutive_aux_merge_failures",
      this->concat_max_consec_fail_);
  std::vector<double> aux_static_flat;
  this->get_parameter("localization/lidar_concat/aux_static_transforms",
                      aux_static_flat);

  if (this->concat_enabled_) {
    if (aux_topics_param.size() != aux_frames_param.size()) {
      // A misconfigured REQUIRED merge must not silently degrade to
      // primary-only. Hard-fail only when the strict path is also set to abort;
      // otherwise warn and disable concat (non-fatal, consistent with
      // abort_on_merge_failure=false).
      if (this->concat_require_all_aux_ &&
          this->concat_abort_on_merge_failure_) {
        RCLCPP_FATAL(this->get_logger(),
                     "lidar_concat: aux_topics size (%zu) != aux_frames size "
                     "(%zu) with require_all_aux=true and "
                     "abort_on_merge_failure=true; refusing to start. Fix the "
                     "config, or set require_all_aux=false / "
                     "abort_on_merge_failure=false.",
                     aux_topics_param.size(), aux_frames_param.size());
        throw std::runtime_error("lidar_concat: aux_topics/aux_frames size "
                                 "mismatch (require_all_aux)");
      }
      RCLCPP_ERROR(this->get_logger(),
                   "lidar_concat: aux_topics size (%zu) != aux_frames size "
                   "(%zu); disabling concat",
                   aux_topics_param.size(), aux_frames_param.size());
      this->concat_enabled_ = false;
    } else if (aux_topics_param.empty()) {
      if (this->concat_require_all_aux_ &&
          this->concat_abort_on_merge_failure_) {
        RCLCPP_FATAL(this->get_logger(),
                     "lidar_concat enabled with no aux_topics, "
                     "require_all_aux=true and abort_on_merge_failure=true; "
                     "refusing to start. Configure aux_topics/aux_frames, or "
                     "set require_all_aux=false / "
                     "abort_on_merge_failure=false.");
        throw std::runtime_error("lidar_concat: enabled but no aux_topics "
                                 "configured (require_all_aux)");
      }
      RCLCPP_WARN(this->get_logger(), "lidar_concat enabled but no aux_topics "
                                      "configured; disabling concat");
      this->concat_enabled_ = false;
    } else {
      for (size_t i = 0; i < aux_topics_param.size(); ++i) {
        auto aux = std::make_unique<AuxLidar>();
        aux->topic = aux_topics_param[i];
        aux->frame = aux_frames_param[i];
        aux->T_primary_aux = Eigen::Matrix4f::Identity();
        aux->extrinsic_cached = false;
        this->aux_lidars_.push_back(std::move(aux));
      }
      RCLCPP_INFO(this->get_logger(),
                  "lidar_concat enabled: %zu aux lidars, time_threshold=%.3fs, "
                  "buffer_size=%zu, FLOAT64 time=%s",
                  this->aux_lidars_.size(), this->concat_time_threshold_,
                  this->concat_buffer_size_,
                  this->concat_float64_time_is_epoch_ns_
                      ? "raw uint64 epoch-ns (explicit opt-in)"
                      : "scan-relative seconds");
      if (this->concat_aux_time_offsets_.size() < this->aux_lidars_.size()) {
        RCLCPP_WARN(
            this->get_logger(),
            "lidar_concat: aux_time_offsets has %zu/%zu entries; missing "
            "entries default to 0.0. "
            "If aux LiDAR clocks have a constant offset vs the primary/IMU "
            "clock, configure "
            "localization/lidar_concat/aux_time_offsets in aux_topics order.",
            this->concat_aux_time_offsets_.size(), this->aux_lidars_.size());
      } else if (this->concat_aux_time_offsets_.size() >
                 this->aux_lidars_.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "lidar_concat: aux_time_offsets has %zu entries for %zu "
                    "aux lidars; extra entries will be ignored",
                    this->concat_aux_time_offsets_.size(),
                    this->aux_lidars_.size());
      }
      for (const auto &a : this->aux_lidars_) {
        RCLCPP_INFO(this->get_logger(), "  aux lidar: topic='%s' frame='%s'",
                    a->topic.c_str(), a->frame.c_str());
      }

      // Split the flat static-transform array (16 row-major doubles per aux, in
      // aux order) into per-aux 4x4 matrices for the offline resolver.
      std::vector<std::vector<double>> aux_static_transforms;
      if (!aux_static_flat.empty()) {
        if (aux_static_flat.size() == 16 * this->aux_lidars_.size()) {
          aux_static_transforms.resize(this->aux_lidars_.size());
          for (size_t i = 0; i < this->aux_lidars_.size(); ++i) {
            aux_static_transforms[i].assign(aux_static_flat.begin() + 16 * i,
                                            aux_static_flat.begin() +
                                                16 * (i + 1));
          }
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "lidar_concat: aux_static_transforms has %zu values, "
                      "expected %zu (16 x %zu aux); ignoring",
                      aux_static_flat.size(), 16 * this->aux_lidars_.size(),
                      this->aux_lidars_.size());
        }
      }

      // Resolve aux extrinsics now, without live TF (URDF > static >
      // TF-at-runtime).
      this->resolveAuxExtrinsicsOffline(aux_static_transforms);
    }
  }

  // A fused/driver-calibrated IMU can explicitly bypass the legacy stationary
  // bias estimator. Its configured orientation is still consumed normally;
  // only the startup bias-learning state machine is skipped and zero residual
  // biases are retained.
  this->declare_parameter<bool>("localization/imu/precalibrated", false);
  this->get_parameter("localization/imu/precalibrated",
                      this->imu_precalibrated_);

  // IMU calibration time (seconds of stationary data to average for
  // bias/gravity when precalibrated=false)
  this->declare_parameter<double>("imu_calibration_time", 3.0);
  this->get_parameter("imu_calibration_time", this->imu_calib_time_);
  // Generic library default is source-neutral. Deployment profiles may enable
  // the guard and provide an exact topic allowlist.
  this->declare_parameter<bool>("localization/imu/require_topic_allowlist",
                                false);
  this->declare_parameter<std::vector<std::string>>(
      "localization/imu/topic_allowlist", std::vector<std::string>{});
  this->get_parameter("localization/imu/require_topic_allowlist",
                      this->imu_require_topic_allowlist_);
  this->get_parameter("localization/imu/topic_allowlist",
                      this->imu_topic_allowlist_);
  // Safety guard: reject IMU samples whose header.frame_id does not match
  // localization/imu_frame. VKS publishes the selected IMU in the CG frame.
  this->declare_parameter<bool>("localization/imu/require_frame_match", true);
  this->get_parameter("localization/imu/require_frame_match",
                      this->imu_require_frame_match_);

  // RTK-driven IMU calibration. When enabled, the first accepted GT odom
  // sample triggers a calibration window in which IMU residuals are computed
  // against the GT pose/twist (no stationary assumption). With the RTK gate
  // enabled, "accepted" means /vks/filtered_odom reported pose covariance
  // within the configured thresholds; disabling the gate for bag replay
  // removes that guarantee. Falls back to stationary calibration if no
  // accepted GT arrives within fallback_timeout.
  this->declare_parameter<bool>("localization/rtk_init/enable", true);
  this->declare_parameter<double>("localization/rtk_init/calib_window", 2.0);
  this->declare_parameter<double>("localization/rtk_init/fallback_timeout",
                                  5.0);
  this->get_parameter("localization/rtk_init/enable", this->rtk_init_enabled_);
  this->get_parameter("localization/rtk_init/calib_window",
                      this->rtk_calib_window_sec_);
  this->get_parameter("localization/rtk_init/fallback_timeout",
                      this->rtk_fallback_timeout_sec_);
  if (this->imu_precalibrated_) {
    RCLCPP_INFO(this->get_logger(),
                "IMU bias calibration: bypassed (precalibrated input)");
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "RTK-driven IMU calibration: %s (window=%.1fs, fallback_timeout=%.1fs)",
        this->rtk_init_enabled_ ? "ENABLED" : "disabled",
        this->rtk_calib_window_sec_, this->rtk_fallback_timeout_sec_);
  }

  // Sensor type for per-point timestamp handling during deskewing
  this->declare_parameter<std::string>("localization/sensor_type", "ouster");
  std::string sensor_type_str;
  this->get_parameter("localization/sensor_type", sensor_type_str);
  if (sensor_type_str == "luminar") {
    this->sensor = gicp_localizer::SensorType::LUMINAR;
  } else if (sensor_type_str == "velodyne") {
    this->sensor = gicp_localizer::SensorType::VELODYNE;
  } else if (sensor_type_str == "hesai") {
    this->sensor = gicp_localizer::SensorType::HESAI;
  } else if (sensor_type_str == "livox") {
    this->sensor = gicp_localizer::SensorType::LIVOX;
  } else if (sensor_type_str == "ouster") {
    this->sensor = gicp_localizer::SensorType::OUSTER;
  } else {
    this->sensor = gicp_localizer::SensorType::UNKNOWN;
    RCLCPP_WARN(this->get_logger(),
                "Unknown localization/sensor_type '%s'; per-point deskew needs "
                "ouster, velodyne, "
                "hesai, livox, or luminar",
                sensor_type_str.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Sensor type: %s", sensor_type_str.c_str());

  // Geometric Observer parameters. Position/orientation gains stay active, but
  // online IMU bias adaptation defaults off for the fused Point One (Atlas) INS
  // path; the initial RTK/stationary calibration still seeds state.b once
  // before propagation.
  this->declare_parameter<double>("odom/geo/Kp", 4.5);
  this->declare_parameter<double>("odom/geo/Kv", 11.25);
  this->declare_parameter<double>("odom/geo/Kq", 4.0);
  this->declare_parameter<double>("odom/geo/Kab", 0.0);
  this->declare_parameter<double>("odom/geo/Kgb", 0.0);
  // Same scan-timeline residual feedback used by perception-ws cleanup:
  // v_next = v_imu + gain * (p_gicp - p_imu) / dt.
  this->declare_parameter<double>("vel_feedback_gain", 0.8);
  this->declare_parameter<std::string>("localization/wheel_speed/topic", "");
  this->declare_parameter<double>("localization/wheel_speed/feedback_gain",
                                  0.5);
  this->declare_parameter<double>("localization/wheel_speed/max_age_s", 0.3);
  this->declare_parameter<bool>(
      "localization/wheel_speed/stationary_gate/enable", false);
  this->declare_parameter<double>(
      "localization/wheel_speed/stationary_gate/enter_speed_mps", 0.2);
  this->declare_parameter<double>(
      "localization/wheel_speed/stationary_gate/exit_speed_mps", 0.5);
  this->declare_parameter<double>("odom/geo/Kz_damping", 5.0);
  this->declare_parameter<double>("odom/geo/abias_max", 5.0);
  this->declare_parameter<double>("odom/geo/gbias_max", 0.5);

  this->get_parameter("odom/geo/Kp", this->geo_Kp_);
  this->get_parameter("odom/geo/Kv", this->geo_Kv_);
  this->get_parameter("odom/geo/Kq", this->geo_Kq_);
  this->get_parameter("odom/geo/Kab", this->geo_Kab_);
  this->get_parameter("odom/geo/Kgb", this->geo_Kgb_);
  this->get_parameter("vel_feedback_gain", this->scan_velocity_feedback_gain_);
  this->get_parameter("localization/wheel_speed/topic",
                      this->wheel_speed_topic_);
  this->get_parameter("localization/wheel_speed/feedback_gain",
                      this->wheel_speed_feedback_gain_);
  this->get_parameter("localization/wheel_speed/max_age_s",
                      this->wheel_speed_max_age_s_);
  this->get_parameter("localization/wheel_speed/stationary_gate/enable",
                      this->wheel_stationary_gate_enabled_);
  this->get_parameter(
      "localization/wheel_speed/stationary_gate/enter_speed_mps",
      this->wheel_stationary_enter_speed_mps_);
  this->get_parameter("localization/wheel_speed/stationary_gate/exit_speed_mps",
                      this->wheel_stationary_exit_speed_mps_);
  this->get_parameter("odom/geo/Kz_damping", this->geo_Kz_damping_);
  this->get_parameter("odom/geo/abias_max", this->geo_abias_max_);
  this->get_parameter("odom/geo/gbias_max", this->geo_gbias_max_);
  if (!std::isfinite(this->scan_velocity_feedback_gain_) ||
      this->scan_velocity_feedback_gain_ < 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "vel_feedback_gain must be finite and >= 0; using 0.8");
    this->scan_velocity_feedback_gain_ = 0.8;
  }
  if (!std::isfinite(this->wheel_speed_feedback_gain_) ||
      this->wheel_speed_feedback_gain_ < 0.0 ||
      this->wheel_speed_feedback_gain_ > 1.0) {
    RCLCPP_WARN(this->get_logger(),
                "wheel-speed feedback gain must be in [0,1]; using 0.5");
    this->wheel_speed_feedback_gain_ = 0.5;
  }
  if (!std::isfinite(this->wheel_speed_max_age_s_) ||
      this->wheel_speed_max_age_s_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "wheel-speed max age must be finite and >0; using 0.3 s");
    this->wheel_speed_max_age_s_ = 0.3;
  }
  if (!std::isfinite(this->wheel_stationary_enter_speed_mps_) ||
      !std::isfinite(this->wheel_stationary_exit_speed_mps_) ||
      this->wheel_stationary_enter_speed_mps_ < 0.0 ||
      this->wheel_stationary_exit_speed_mps_ <
          this->wheel_stationary_enter_speed_mps_) {
    RCLCPP_WARN(this->get_logger(),
                "stationary wheel gate requires 0 <= enter <= exit; using "
                "0.2/0.5 m/s");
    this->wheel_stationary_enter_speed_mps_ = 0.2;
    this->wheel_stationary_exit_speed_mps_ = 0.5;
  }

  // P3: delta-form observer correction. The GICP measurement and its IMU
  // prior are both stamped at the scan's median point time, 0.1-0.3 s before
  // the correction is applied (half sweep + queueing + GICP solve). Legacy
  // behavior pulls the CURRENT state toward that stale absolute pose, which
  // is a systematic backward/yaw-lag drag during turns (accepted-frame gt_err
  // scaled with yaw rate on run 12). Delta form instead applies
  // T_corr = T_meas * inv(T_prior) — the time-free IMU-drift correction — to
  // the current state, so a perfect IMU/GICP agreement produces a ZERO
  // correction regardless of latency. false = legacy absolute-target observer.
  this->declare_parameter<bool>("odom/geo/delta_correction", true);
  this->get_parameter("odom/geo/delta_correction", this->geo_delta_correction_);

  // Observer-correction stability bounds (P2#1).
  this->declare_parameter<double>("odom/geo/observer_dt_max", 0.15);
  this->declare_parameter<double>("odom/geo/max_pos_correction", 0.0);
  this->declare_parameter<double>("odom/geo/max_vel_correction", 0.0);
  this->declare_parameter<double>("odom/geo/max_state_speed", 0.0);
  // P1 yaw-safety fix #3: per-update ORIENTATION clamps (position/velocity
  // already had them). Yaw clamp ON by default — the failure mode it bounds
  // (one bad accepted scan yanking heading tens of degrees) is exactly the
  // runs-19/20 signature. 0 disables.
  this->declare_parameter<double>("odom/geo/max_yaw_correction_deg", 5.0);
  this->declare_parameter<double>("odom/geo/max_rot_correction_deg", 0.0);
  this->get_parameter("odom/geo/observer_dt_max", this->geo_observer_dt_max_);
  this->get_parameter("odom/geo/max_pos_correction",
                      this->geo_max_pos_correction_);
  this->get_parameter("odom/geo/max_vel_correction",
                      this->geo_max_vel_correction_);
  this->get_parameter("odom/geo/max_state_speed", this->geo_max_state_speed_);
  this->get_parameter("odom/geo/max_yaw_correction_deg",
                      this->geo_max_yaw_correction_deg_);
  this->get_parameter("odom/geo/max_rot_correction_deg",
                      this->geo_max_rot_correction_deg_);
  if (this->geo_observer_dt_max_ <= 0.0) {
    this->geo_observer_dt_max_ =
        0.15; // guard against a non-positive cap disabling all corrections
  }
  if (!std::isfinite(this->geo_max_state_speed_) ||
      this->geo_max_state_speed_ < 0.0) {
    RCLCPP_WARN(this->get_logger(),
                "odom/geo/max_state_speed=%.3f invalid; disabling speed clamp",
                this->geo_max_state_speed_);
    this->geo_max_state_speed_ = 0.0;
  }
  // Time/speed-based dead-reckoning covariance growth (P3).
  this->declare_parameter<double>("odom/geo/dr_cov_time_rate", 0.5);
  this->declare_parameter<double>("odom/geo/dr_cov_dist_frac", 0.05);
  this->get_parameter("odom/geo/dr_cov_time_rate", this->dr_cov_time_rate_);
  this->get_parameter("odom/geo/dr_cov_dist_frac", this->dr_cov_dist_frac_);

  // Debug parameters
  this->declare_parameter<bool>("localization/debug/enable_jump_log", true);
  this->declare_parameter<bool>("localization/debug/verbose_scan_log", false);
  this->declare_parameter<bool>("localization/debug/small_gicp_lm_debug",
                                false);
  this->declare_parameter<bool>("localization/debug/nano_gicp_lm_debug",
                                false); // deprecated compatibility alias
  this->declare_parameter<double>(
      "localization/debug/jump_trans_m",
      30.0); // [P3 FIX 2026-07-10] yaml-aligned (1.0 mass-rejected at speed)
  this->declare_parameter<double>("localization/debug/jump_rot_deg",
                                  30.0); // [P3 FIX 2026-07-10] yaml-aligned
  this->declare_parameter<bool>("localization/verbose", false);

  this->get_parameter("localization/debug/enable_jump_log",
                      this->debug_jump_log_enabled_);
  this->get_parameter("localization/debug/verbose_scan_log",
                      this->debug_verbose_scan_log_);
  this->get_parameter("localization/debug/small_gicp_lm_debug",
                      this->debug_lm_print_);
  bool legacy_nano_lm_debug = false;
  this->get_parameter("localization/debug/nano_gicp_lm_debug",
                      legacy_nano_lm_debug);
  this->debug_lm_print_ = this->debug_lm_print_ || legacy_nano_lm_debug;
  this->get_parameter("localization/debug/jump_trans_m",
                      this->debug_jump_trans_m_);
  this->get_parameter("localization/debug/jump_rot_deg",
                      this->debug_jump_rot_deg_);

  // Speed/scan_dt-aware jump-gate scaling (P2#2). 0 reproduces the fixed
  // thresholds.
  this->declare_parameter<double>("localization/jump/trans_speed_scale", 1.5);
  this->declare_parameter<double>("localization/jump/vertical_m", 0.0);
  this->declare_parameter<double>("localization/jump/rot_dt_scale_deg", 60.0);
  // P1 yaw-safety: yaw split out of the 3D rotation jump gate. The generic
  // 30 + 60*dt envelope admits re-acquisition but is physically absurd as a
  // YAW budget on a ground vehicle; yaw gets its own tight limit with an
  // absolute cap scan_dt scaling can never lift into the tens of degrees.
  this->declare_parameter<double>("localization/jump/yaw_max_deg", 10.0);
  this->declare_parameter<double>("localization/jump/yaw_dt_scale_deg", 10.0);
  this->declare_parameter<double>("localization/jump/yaw_total_max_deg", 15.0);
  this->get_parameter("localization/jump/trans_speed_scale",
                      this->jump_trans_speed_scale_);
  this->get_parameter("localization/jump/vertical_m", this->jump_vertical_m_);
  this->get_parameter("localization/jump/rot_dt_scale_deg",
                      this->jump_rot_dt_scale_deg_);
  this->get_parameter("localization/jump/yaw_max_deg", this->jump_yaw_max_deg_);
  this->get_parameter("localization/jump/yaw_dt_scale_deg",
                      this->jump_yaw_dt_scale_deg_);
  this->get_parameter("localization/jump/yaw_total_max_deg",
                      this->jump_yaw_total_max_deg_);

  this->get_parameter("localization/verbose", this->verbose_);

  // `verbose_` and `debug_verbose_scan_log_` gate their high-rate call sites
  // directly. Do not lower the whole logger to WARN here: production mode
  // still needs bounded INFO lifecycle evidence (map ready, node initialized,
  // recovery transitions and the final queue/drop summary).

  RCLCPP_INFO(
      this->get_logger(),
      "Preprocessing config: crop_size=%.2f, voxel_filter=%s, voxel_res=%.2f, "
      "point_budget=%lu (%s, azimuth_bins=%lu, range_bins=%lu)",
      this->crop_size_, this->vf_use_ ? "ENABLED" : "DISABLED", this->vf_res_,
      this->scan_max_points_,
      this->scan_point_budget_spatial_balancing_ ? "spatial" : "ordered",
      this->scan_point_budget_azimuth_bins_,
      this->scan_point_budget_range_bins_);
  RCLCPP_INFO(this->get_logger(),
              "IMU config: deskew=%s, precalibrated=%s, gravity=%.2f, "
              "buffer_size=%d, trajectory_knot=%.1fms",
              this->deskew_ ? "ENABLED" : "DISABLED",
              this->imu_precalibrated_ ? "yes" : "no", this->gravity_,
              this->imu_buffer_size_, 1e3 * this->deskew_knot_interval_s_);
  RCLCPP_INFO(this->get_logger(),
              "Geometric Observer: Kp=%.2f, Kv=%.2f, Kq=%.2f, Kab=%.2f, "
              "Kgb=%.2f, scan_vel_feedback=%.2f",
              this->geo_Kp_, this->geo_Kv_, this->geo_Kq_, this->geo_Kab_,
              this->geo_Kgb_, this->scan_velocity_feedback_gain_);
  RCLCPP_INFO(this->get_logger(),
              "Wheel-speed anchor: topic='%s' gain=%.2f max_age=%.3fs, "
              "stationary_gate=%s [enter<=%.2f, exit>=%.2f]m/s, "
              "latency_distance_anchor=%s",
              this->wheel_speed_topic_.empty()
                  ? "disabled"
                  : this->wheel_speed_topic_.c_str(),
              this->wheel_speed_feedback_gain_, this->wheel_speed_max_age_s_,
              this->wheel_stationary_gate_enabled_ ? "on" : "off",
              this->wheel_stationary_enter_speed_mps_,
              this->wheel_stationary_exit_speed_mps_,
              this->output_wheel_distance_anchor_enabled_ ? "on" : "off");
  RCLCPP_INFO(this->get_logger(), "Localization mode: %s",
              this->imu_only_mode_ ? "IMU-only (GICP disabled)" : "GICP + IMU");
  RCLCPP_INFO(
      this->get_logger(),
      "GICP target: local_map=%s radius=%.1fm rebuild=%.1fm lead=%.2fs/%.1fm "
      "grid=%.1fm min_points=%zu builder_threads=%d",
      this->local_map_enabled_ ? "ENABLED" : "disabled",
      this->local_map_radius_m_, this->local_map_rebuild_distance_m_,
      this->local_map_lead_time_s_, this->local_map_max_lead_m_,
      this->local_map_grid_cell_size_m_, this->local_map_min_points_,
      this->local_map_builder_threads_);
  RCLCPP_INFO(this->get_logger(),
              "GICP rejection: fitness>%.3f, large_jump=%s, hessian_cond>%.2e "
              "AND (fitness>%.3f OR trans>%.2fm OR rot>%.2fdeg) (%s)",
              this->gicp_fitness_reject_threshold_,
              this->gicp_reject_large_jumps_ ? "on" : "off",
              this->gicp_hessian_cond_max_, this->gicp_hessian_fitness_warn_,
              this->gicp_hessian_trans_warn_m_,
              this->gicp_hessian_rot_warn_deg_,
              this->gicp_hessian_cond_max_ > 0.0 ? "on" : "disabled");
  RCLCPP_INFO(this->get_logger(),
              "GT recovery: %s (min consecutive failures=%d)",
              this->gt_recovery_enabled_ ? "ENABLED" : "DISABLED",
              this->gt_recovery_min_consecutive_failures_);
  RCLCPP_INFO(
      this->get_logger(),
      "Observer stability bounds: dt<=%.3fs pos_step<=%.2fm vel_step<=%.2fm/s "
      "state_speed<=%.1fm/s",
      this->geo_observer_dt_max_, this->geo_max_pos_correction_,
      this->geo_max_vel_correction_, this->geo_max_state_speed_);
  RCLCPP_INFO(this->get_logger(),
              "Debug logs: jump_log=%s thresholds=[%.2fm, %.1fdeg]",
              this->debug_jump_log_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_trans_m_, this->debug_jump_rot_deg_);
  RCLCPP_INFO(this->get_logger(),
              "Debug detail: verbose_scan_log=%s small_gicp_lm_debug=%s",
              this->debug_verbose_scan_log_ ? "ENABLED" : "DISABLED",
              this->debug_lm_print_ ? "ENABLED" : "DISABLED");
}
