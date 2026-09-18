#include "gicp_interface/detail/localizer_utils.hpp"
#include "gicp_interface/registration_policy.hpp"
#include "gicp_interface/scan_velocity_feedback.hpp"

using gicp_localizer::detail::deltaTranslationNorm;
using gicp_localizer::detail::matrixFinite;
using gicp_localizer::detail::poseSummary;
using gicp_localizer::detail::rotationDistanceDeg;
using gicp_localizer::detail::scalarSummary;

bool gicp_localizer::GicpLocalizer::performLocalization() {

  RCLCPP_DEBUG(this->get_logger(),
               "performLocalization: Acquiring mutex lock...");
  std::lock_guard<std::mutex> lock(this->pose_mutex);
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Mutex acquired");

  // Set source cloud
  RCLCPP_DEBUG(this->get_logger(),
               "performLocalization: Setting input source (%lu points)...",
               this->current_scan->points.size());
  this->gicp.setInputSource(this->current_scan);
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Input source set");

  // Align using IMU-based prior as initial guess (if deskewing is enabled)
  // align() requires an output cloud parameter (PCL API), but we never use the
  // transformed cloud — LsqRegistration skips the fill, so this stays empty.
  pcl::PointCloud<PointType> aligned_scratch;

  // When the cloud was actually placed in the world frame (main deskew path
  // and the transforming fallbacks), GICP's initial guess is Identity and the
  // final pose = T_corr * T_prior. Otherwise the points are still in the
  // lidar frame, so seed/solve in map<-lidar and convert the optimizer output
  // back to map<-base. [REVIEW FIX 2026-07-08 P2] Keyed on the per-scan
  // scan_in_world_frame_ flag set by deskewPointcloud(), NOT on deskew_:
  // several deskew fallback branches return a sensor-frame cloud while
  // deskew_ is true, and registering that cloud under the world-frame
  // assumption produced wrong-basin candidates at startup / IMU gaps.
  const Eigen::Matrix4f T_base_lidar = this->extrinsics.baselink2lidar_T;
  const Eigen::Matrix4f T_lidar_base = T_base_lidar.inverse();
  // PR#6: plain if-assignment instead of a ternary mixing two different
  // Eigen expression types (CwiseNullaryOp vs Product) — the ternary broke
  // package builds and had to be hot-patched in every replay worktree.
  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  if (!this->scan_in_world_frame_) {
    initial_guess = this->T_prior * T_base_lidar;
  }
  Eigen::Matrix4f guess_pose_map = this->T_prior;
  const bool planar_requested = this->gicp_dof_mode_ == "planar";
  const bool planar_active =
      planar_requested && static_cast<int>(this->accepted_gicp_count_.load(
                              std::memory_order_relaxed)) >=
                              this->gicp_planar_warmup_accepted_scans_;
  bool gicp_position_seed_applied = false;
  double gicp_position_seed_step_m = 0.0;
  double track_z_target_m = std::numeric_limits<double>::quiet_NaN();

  // Optional Atlas translation INITIAL GUESS for GICP. This deliberately
  // does not modify basePose, observer state, T_prior or the published pose:
  // point-cloud registration must still produce a supported candidate and
  // pass the independent wrong-basin gate. It only places the optimizer in
  // the correct local basin when dead-reckoned position has drifted.
  if (this->ins_prior_gicp_position_seed_blend_ > 0.0 &&
      this->gt_odom_enabled_ && this->gt_odom_received_.load()) {
    const double seed_stamp = (this->t_prior_stamp_ > 0.0)
                                  ? this->t_prior_stamp_
                                  : this->scan_stamp.seconds();
    GtSample ins_seed;
    if (this->getGtPoseAt(seed_stamp, ins_seed) &&
        (!this->ins_prior_require_rtk_ || this->gtSampleIsRtkFixed(ins_seed))) {
      Eigen::Vector3f ins_p;
      Eigen::Quaternionf ins_q;
      if (this->composeGtPoseInBase(ins_seed, ins_p, ins_q)) {
        Eigen::Vector3f seed_delta =
            static_cast<float>(this->ins_prior_gicp_position_seed_blend_) *
            (ins_p - this->T_prior.block<3, 1>(0, 3));
        const double raw_step_m = static_cast<double>(seed_delta.norm());
        if (this->ins_prior_gicp_position_seed_max_step_m_ > 0.0 &&
            raw_step_m > this->ins_prior_gicp_position_seed_max_step_m_) {
          seed_delta *= static_cast<float>(
              this->ins_prior_gicp_position_seed_max_step_m_ / raw_step_m);
        }
        if (seed_delta.allFinite()) {
          initial_guess.block<3, 1>(0, 3) += seed_delta;
          gicp_position_seed_step_m = static_cast<double>(seed_delta.norm());
          gicp_position_seed_applied = gicp_position_seed_step_m > 0.0;
        }
      }
    }
  }
  guess_pose_map = this->scan_in_world_frame_ ? (initial_guess * this->T_prior)
                                              : (initial_guess * T_lidar_base);

  // A planar solve needs a map-height seed before z is fixed. Initialization
  // calibrates the static map_z - ttl_z datum offset. Thereafter the
  // authoritative TTL contributes only its relative
  // elevation profile; no VKS/GNSS sample is consumed here. Apply the target
  // to the optimizer initial guess rather than mutating T_prior after deskew,
  // which would relabel a cloud already placed with the original prior.
  if (planar_active && this->track_elevation_enabled_ &&
      this->track_constraint_ &&
      this->track_z_hold_valid_.load(std::memory_order_acquire)) {
    const double held_z = this->track_z_hold_m_.load(std::memory_order_relaxed);
    const auto elevation =
        this->track_constraint_->projectElevation(guess_pose_map);
    if (elevation.valid && !this->track_z_offset_valid_) {
      // Transition from a zero-Z pit TTL to the main-track elevation profile
      // without a vertical step. The held height is the one-shot
      // initialization/recovery datum.
      this->track_z_offset_m_ = held_z - elevation.z;
      this->track_z_offset_valid_ = true;
      RCLCPP_INFO(
          this->get_logger(),
          "Calibrated static TTL elevation datum at first valid profile: "
          "map_z - ttl_z = %.3fm (line=%s, center_distance=%.2fm)",
          this->track_z_offset_m_, elevation.line_name.c_str(),
          elevation.center_distance_m);
    }
    if (elevation.valid && this->track_z_offset_valid_) {
      track_z_target_m = elevation.z + this->track_z_offset_m_;
    } else {
      // Pit/legacy TTLs may intentionally omit elevation (all Z values are
      // zero). Hold the product datum until a valid profile is
      // available instead of letting IMU double integration drift vertically.
      track_z_target_m = held_z;
    }
    if (this->scan_in_world_frame_) {
      initial_guess(2, 3) +=
          static_cast<float>(track_z_target_m - guess_pose_map(2, 3));
    } else {
      Eigen::Matrix4f target_base_pose = guess_pose_map;
      target_base_pose(2, 3) = static_cast<float>(track_z_target_m);
      initial_guess = target_base_pose * T_base_lidar;
    }
    guess_pose_map = this->scan_in_world_frame_
                         ? Eigen::Matrix4f(initial_guess * this->T_prior)
                         : Eigen::Matrix4f(initial_guess * T_lidar_base);
  }

  double guess_from_last_trans = 0.0;
  double guess_from_last_vertical = 0.0;
  double guess_from_last_rot_deg = 0.0;
  if (this->last_gicp_valid_) {
    guess_from_last_trans =
        deltaTranslationNorm(this->last_gicp_pose_, guess_pose_map);
    guess_from_last_vertical = std::abs(static_cast<double>(
        guess_pose_map(2, 3) - this->last_gicp_pose_(2, 3)));
    guess_from_last_rot_deg =
        rotationDistanceDeg(this->last_gicp_pose_, guess_pose_map);
  }

  double scan_dt = 0.0;
  if (this->last_gicp_valid_) {
    scan_dt = (this->scan_stamp - this->last_gicp_stamp_).seconds();
  }
  float speed_est = 0.0f;
  Eigen::Vector3f velocity_est_world = Eigen::Vector3f::Zero();
  {
    std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
    velocity_est_world = this->state.v.lin.w;
    speed_est = velocity_est_world.norm();
    this->registration_observer_epoch_ = this->observer_epoch_;
  }
  // Target selection happens only after the complete map has been prepared.
  // A background crop may become active here, but this scan never waits for
  // it: startup, recovery, and stale-crop cases use the complete target.
  this->updateLocalMapTarget(guess_pose_map, velocity_est_world);

  // Time-match the proprioceptive body-x speed to the same median point time
  // as T_prior. The wheel callback may be ahead of the scan worker, hence the
  // reverse lookup in a short timestamped buffer instead of a latest-value
  // read. Never consume a sample newer than the scan prior timestamp.
  double wheel_speed_mps = -1.0;
  double wheel_speed_age_s = std::numeric_limits<double>::infinity();
  bool wheel_speed_fresh = false;
  if (!this->wheel_speed_topic_.empty() && this->t_prior_stamp_ > 0.0) {
    std::lock_guard<std::mutex> wheel_lock(this->wheel_speed_mtx_);
    for (auto it = this->wheel_speed_buffer_.rbegin();
         it != this->wheel_speed_buffer_.rend(); ++it) {
      if (it->first <= this->t_prior_stamp_) {
        wheel_speed_age_s = this->t_prior_stamp_ - it->first;
        if (wheel_speed_age_s <= this->wheel_speed_max_age_s_) {
          wheel_speed_mps = it->second;
          wheel_speed_fresh = true;
        }
        break;
      }
    }
  }
  const bool stationary_gate_was_active = this->wheel_stationary_gate_active_;
  this->wheel_stationary_gate_active_ =
      this->wheel_stationary_gate_enabled_ && this->last_gicp_valid_ &&
      updateStationaryMotionGate(stationary_gate_was_active, wheel_speed_fresh,
                                 wheel_speed_mps,
                                 this->wheel_stationary_enter_speed_mps_,
                                 this->wheel_stationary_exit_speed_mps_);
  const bool wheel_stationary_hold = this->wheel_stationary_gate_active_;
  if (wheel_stationary_hold != stationary_gate_was_active) {
    RCLCPP_INFO(this->get_logger(),
                "Wheel stationary gate %s at %.3f m/s (enter<=%.3f, "
                "exit>=%.3f)",
                wheel_stationary_hold ? "ENGAGED" : "RELEASED", wheel_speed_mps,
                this->wheel_stationary_enter_speed_mps_,
                this->wheel_stationary_exit_speed_mps_);
  }

  const PreSolveGuessGate pre_solve_gate = evaluatePreSolveGuessGate(
      this->last_gicp_valid_, guess_from_last_trans, guess_from_last_vertical,
      guess_from_last_rot_deg, static_cast<double>(speed_est), scan_dt,
      this->debug_jump_trans_m_, this->jump_trans_speed_scale_,
      this->jump_vertical_m_, this->debug_jump_rot_deg_,
      this->jump_rot_dt_scale_deg_);
  const bool pre_solve_guess_rejected = pre_solve_gate.rejected;

  // ---- Ground-vehicle registration constraints (yaw-defect fix) ----
  // DoF mask: fix roll/pitch (4dof), full attitude (3dof), or roll/pitch/z
  // (planar) to the initial
  // guess INSIDE the optimizer — the IMU prior attitude cannot be tens of
  // degrees wrong over one scan gap, so wrong-basin attitude updates are
  // removed at the source instead of gated after the fact. Every
  // full6dofEveryN-th scan runs unconstrained so roll/pitch re-anchor to the
  // map (bounds slow gyro-drift accumulation in the fixed axes).
  bool dof_full6_this_scan = (this->gicp_dof_mode_ == "6dof");
  if (!planar_requested && !dof_full6_this_scan &&
      this->gicp_full6dof_every_n_ > 0 &&
      (++this->dof_scan_counter_ % this->gicp_full6dof_every_n_) == 0) {
    dof_full6_this_scan = true;
  }
  const bool fix_rp = !dof_full6_this_scan &&
                      (this->gicp_dof_mode_ == "4dof" ||
                       this->gicp_dof_mode_ == "3dof" || planar_requested);
  const bool fix_yaw_dof =
      !dof_full6_this_scan && this->gicp_dof_mode_ == "3dof";
  this->gicp.setDoFMask(fix_rp, fix_rp, fix_yaw_dof, false, false,
                        planar_active);
  // Soft attitude prior toward the IMU-integrated initial guess (P1 deep
  // cause: the optimizer had NO IMU attitude term, so repeated map structure
  // could pull yaw into a plausible wrong basin unopposed).
  if (this->gicp_prior_yaw_info_ > 0.0 ||
      this->gicp_prior_rollpitch_info_ > 0.0) {
    const Eigen::Matrix3d R_target =
        initial_guess.block<3, 3>(0, 0).cast<double>();
    this->gicp.setRotationPrior(
        R_target, Eigen::Vector3d(this->gicp_prior_rollpitch_info_,
                                  this->gicp_prior_rollpitch_info_,
                                  this->gicp_prior_yaw_info_));
  } else {
    this->gicp.clearRotationPrior();
  }

  RCLCPP_DEBUG(this->get_logger(),
               "performLocalization: Starting GICP alignment...");
  const RegistrationSolveLimits solve_limits = selectRegistrationSolveLimits(
      this->accepted_gicp_count_.load(std::memory_order_relaxed),
      this->gicp_startup_accepted_scans_, this->consecutive_failures_,
      this->failure_streak_had_timeout_, this->gicp_max_optimization_time_ms_,
      this->gicp_startup_max_optimization_time_ms_, this->gicp_max_iter_,
      this->gicp_startup_max_iter_);
  const double active_gicp_budget_ms = solve_limits.optimization_time_ms;
  const int active_gicp_max_iterations = solve_limits.max_iterations;
  this->gicp.setMaximumOptimizationTimeMs(active_gicp_budget_ms);
  this->gicp.setMaximumIterations(active_gicp_max_iterations);
  double elapsed_ms = 0.0;
  size_t gicp_iterations = 0;
  int gicp_inner_trials = 0;
  int gicp_rejected_inner_trials = 0;
  double source_tree_ms = 0.0;
  double source_covariance_ms = 0.0;
  double target_covariance_ms = 0.0;
  double optimizer_ms = 0.0;
  double fitness_score = std::numeric_limits<double>::infinity();
  bool backend_timed_out = false;
  bool gicp_budget_limited = false;
  bool converged_precheck = false;
  double final_error = std::numeric_limits<double>::infinity();
  int num_correspondences = 0;
  Eigen::Matrix4f optimizer_solution = initial_guess;
  Eigen::Matrix<double, 6, 6> final_hessian =
      Eigen::Matrix<double, 6, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  const char *backend_timeout_stage = "pre_solve_guess";

  if (!pre_solve_guess_rejected) {
    const auto start = std::chrono::high_resolution_clock::now();
    this->gicp.align(aligned_scratch, initial_guess);
    const auto end = std::chrono::high_resolution_clock::now();
    RCLCPP_DEBUG(this->get_logger(),
                 "performLocalization: GICP alignment completed");
    elapsed_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count() /
        1000.0;
    gicp_iterations = this->gicp.getRegistrationResult().iterations;
    gicp_inner_trials = this->gicp.getInnerTrials();
    gicp_rejected_inner_trials = this->gicp.getRejectedInnerTrials();
    source_tree_ms = this->gicp.getSourceTreeMs();
    source_covariance_ms = this->gicp.getSourceCovarianceMs();
    target_covariance_ms = this->gicp.getTargetCovarianceMs();
    optimizer_ms = this->gicp.getOptimizerMs();
    fitness_score = this->gicp.getFitnessScore();
    backend_timed_out = this->gicp.hasTimedOut();
    backend_timeout_stage = this->gicp.getTimeoutStage();
    gicp_budget_limited = this->gicp.wasBudgetLimited();
    converged_precheck = this->gicp.hasConverged();
    final_error = this->gicp.getFinalError();
    num_correspondences = this->gicp.num_correspondences;
    optimizer_solution = this->gicp.getFinalTransformation();
    final_hessian = this->gicp.getFinalHessian();
  }

  const bool hard_runtime_exceeded =
      elapsed_ms > this->gicp_hard_result_max_age_ms_;
  const bool gicp_timed_out = backend_timed_out || hard_runtime_exceeded;
  const char *timeout_stage =
      hard_runtime_exceeded ? "hard_result_age" : backend_timeout_stage;
  double fitness_score_final = fitness_score;
  if (!pre_solve_guess_rejected && !converged_precheck) {
    // Yaw-defect fix (P2a): the cached score reflects the LAST LINEARIZATION
    // pose; on non-converged scans that lags the applied pose by one LM step,
    // and the "effectively converged" acceptance below would judge a bad
    // final candidate by a stale, flattering number. [P3 FIX 2026-07-10]
    // OBSOLETE since the backend's final raw re-linearization (2026-07-08):
    // both getters now return the same final-pose value, so the max() is a
    // no-op kept only to fill fitness_score_final for the debug fields.
    fitness_score_final = this->gicp.getFitnessScoreAtFinal();
    fitness_score = std::max(fitness_score, fitness_score_final);
  }
  this->last_fitness_score_ = fitness_score;
  const bool converged = !pre_solve_guess_rejected && converged_precheck;
  const double correspondence_ratio =
      this->current_scan->points.empty()
          ? 0.0
          : static_cast<double>(num_correspondences) /
                static_cast<double>(this->current_scan->points.size());
  // [P2 FIX 2026-07-10] Bring the Hessian into ONE tangent convention before
  // ANY analysis. small_gicp's H lives in the right-multiplicative tangent of
  // the optimizer's T at the SOURCE points: on the world path (T ~= I, points
  // in world coordinates) that coincides with the world-origin left tangent
  // the re-centering machinery (skew(p_prior) lever arm) and the calibrated
  // hessianCondMax/relFloor thresholds assume. On the deskew-FALLBACK path
  // the source cloud stays in LiDAR coordinates and T = map<-lidar: the raw H
  // is sensor-centered, and re-centering it with the map-position lever arm
  // as if it were world-origin produced meaningless eigen-axes — the
  // degeneracy projection could suppress well-constrained axes and pass
  // degenerate ones, exactly on the fragile frames (IMU gaps, unsupported
  // timestamps). Transform: delta_left = Ad_T * delta_right  =>
  // H_left = Ad_T^{-T} * H * Ad_T^{-1}, with [w;t] ordering and
  // Ad_{T^-1} = [[R^T, 0], [-R^T*skew(t), R^T]]. Verified numerically against
  // synthetic point-pair Hessians in both parameterizations (rel. err 3e-16).
  Eigen::Matrix<double, 6, 6> analysis_hessian = final_hessian;
  if (!this->scan_in_world_frame_ && final_hessian.allFinite() &&
      matrixFinite(optimizer_solution)) {
    const Eigen::Matrix3d R_sol =
        optimizer_solution.block<3, 3>(0, 0).cast<double>();
    const Eigen::Vector3d t_sol =
        optimizer_solution.block<3, 1>(0, 3).cast<double>();
    Eigen::Matrix3d t_hat;
    t_hat << 0.0, -t_sol.z(), t_sol.y(), t_sol.z(), 0.0, -t_sol.x(), -t_sol.y(),
        t_sol.x(), 0.0;
    Eigen::Matrix<double, 6, 6> Ad_inv = Eigen::Matrix<double, 6, 6>::Zero();
    Ad_inv.block<3, 3>(0, 0) = R_sol.transpose();
    Ad_inv.block<3, 3>(3, 0) = -R_sol.transpose() * t_hat;
    Ad_inv.block<3, 3>(3, 3) = R_sol.transpose();
    analysis_hessian = Ad_inv.transpose() * final_hessian * Ad_inv;
  }
  const double hessian_condition = hessianConditionProxy(analysis_hessian);
  const Eigen::Matrix4f candidate_pose =
      this->scan_in_world_frame_ ? (optimizer_solution * this->T_prior)
                                 : (optimizer_solution * T_lidar_base);
  const bool candidate_pose_valid = matrixFinite(candidate_pose);

  // --- P1: per-map fitness baseline (rolling median of accepted-frame
  // fitness). Absolute fitness thresholds calibrated on a same-run map (floor
  // 0.03-0.06) are meaningless on a cross-run map (floor ~0.27); gates below
  // operate on fitness_ratio = fitness / baseline. ratio stays -1 (gates inert)
  // until minSamples accepted frames have been observed.
  double fitness_baseline = -1.0;
  double fitness_ratio = -1.0;
  if (this->fitness_baseline_enable_ &&
      static_cast<int>(this->fitness_history_.size()) >=
          this->fitness_baseline_min_samples_) {
    std::vector<double> tmp(this->fitness_history_.begin(),
                            this->fitness_history_.end());
    const size_t mid = tmp.size() / 2;
    std::nth_element(tmp.begin(), tmp.begin() + mid, tmp.end());
    fitness_baseline = tmp[mid];
  } else if (this->fitness_baseline_enable_ &&
             this->fitness_baseline_seed_ > 0.0) {
    // Warm-up seed (review fix): without it the ratio gates and yaw veto are
    // inert for the first minSamples accepted frames — a replay that starts
    // mid-turn is unprotected exactly when it is most fragile. Seed with the
    // expected per-map floor (read it off the previous run's scorecard,
    // scripts/analyze_scan_debug_log.py); the rolling median takes over once
    // warmed up.
    fitness_baseline = this->fitness_baseline_seed_;
  }
  if (fitness_baseline > 1e-9 && std::isfinite(fitness_score)) {
    fitness_ratio = fitness_score / fitness_baseline;
  }

  // --- P1: degeneracy-aware partial update + yaw-consistency veto.
  // Eigen-projection engages when the (calibrated) condition proxy crosses
  // hessianCondMax; the yaw veto engages independently on low-confidence
  // matches (fitness_ratio above yawGate/fitnessRatio). Either way the scan is
  // not binary-rejected: the correction is projected and the IMU prior kept
  // along untrusted directions.
  const bool eigen_projection_wanted =
      this->degen_partial_update_enable_ && candidate_pose_valid &&
      // [P2 FIX 2026-07-10f] Guard on the HESSIAN being finite, not the
      // condition: hessianConditionProxy now returns +inf for a finite but
      // RANK-DEFICIENT hessian, and that is exactly when the eigen-projection
      // must engage (it identifies the null axes and keeps the IMU prior
      // along them). Non-finite hessians are force-rejected in the gate
      // chain, so they never reach the applied pose either way.
      this->gicp_hessian_cond_max_ > 0.0 && analysis_hessian.allFinite() &&
      hessian_condition > this->gicp_hessian_cond_max_;
  // NOTE: yawGate is deliberately INDEPENDENT of degeneracy/partialUpdate —
  // disabling partial updates (legacy binary hessian gate) must not silently
  // disable the yaw-consistency veto.
  //
  // TWO-TIER YAW VETO (P1 yaw-safety fix, runs 19/20: raw GICP rotation
  // proposals up to 45-47 deg were accepted with plausible fitness, producing
  // 78-98 deg heading errors vs RTK):
  //   * SOFT tier (yawGate/maxCorrDeg, default 1.5 deg): armed only on
  //     low-confidence matches (fitness_ratio > yawGate/fitnessRatio) — the
  //     original wrong-basin ENTRY veto.
  //   * HARD tier (yawGate/hardMaxCorrDeg, default 8 deg): UNCONDITIONAL,
  //     independent of fitness ratio. T_prior carries the IMU-integrated yaw,
  //     whose error over one scan gap is <0.1 deg; a ground vehicle cannot
  //     make the IMU wrong by 8+ deg in 0.1-0.3 s, so any such GICP yaw
  //     "correction" is a wrong basin regardless of how plausible its fitness
  //     looks. The yaw component is dropped (IMU yaw kept); translation is
  //     still gated separately below.
  const bool yaw_soft_armed = candidate_pose_valid && this->yaw_gate_enable_ &&
                              fitness_ratio > 0.0 &&
                              fitness_ratio > this->yaw_gate_fitness_ratio_;
  const bool yaw_hard_armed = candidate_pose_valid && this->yaw_gate_enable_ &&
                              this->yaw_gate_hard_max_corr_deg_ > 0.0;
  double yaw_veto_threshold_deg = 0.0; // 0 = veto disabled in the projector
  if (yaw_soft_armed) {
    yaw_veto_threshold_deg = this->yaw_gate_max_corr_deg_;
  } else if (yaw_hard_armed) {
    yaw_veto_threshold_deg = this->yaw_gate_hard_max_corr_deg_;
  }
  const bool yaw_veto_wanted = yaw_soft_armed || yaw_hard_armed;
  // [P2 FIX 2026-07-10h] rp clamp is UNCONDITIONAL (like the hard yaw veto):
  // it must run on the full-6DoF re-anchor scans, which are exactly the ones
  // where no other attitude protection exists.
  const bool rp_clamp_wanted =
      candidate_pose_valid && this->gicp_rp_hard_max_corr_deg_ > 0.0;
  DegeneracyProjection degen;
  degen.projected_pose = candidate_pose;
  if (eigen_projection_wanted || yaw_veto_wanted || rp_clamp_wanted) {
    degen = projectDegenerateDelta(
        analysis_hessian, this->T_prior, candidate_pose,
        eigen_projection_wanted, this->degen_full6d_,
        this->degen_coupling_length_m_, this->degen_rel_floor_6d_,
        this->degen_rel_floor_rot_, this->degen_rel_floor_trans_,
        yaw_veto_threshold_deg,
        rp_clamp_wanted ? this->gicp_rp_hard_max_corr_deg_ : 0.0);
  }
  // REVIEW FIX: require the projected pose to be finite before adopting it —
  // final_candidate reaches basePose/current_pose ahead of the downstream
  // gicp_valid finiteness guard, so a (pathological) NaN from the projection
  // would otherwise poison the next scan's prior.
  Eigen::Matrix4f final_candidate =
      (degen.valid && degen.modified && matrixFinite(degen.projected_pose))
          ? degen.projected_pose
          : candidate_pose;

  // A stopped vehicle has a stronger motion constraint than scan matching:
  // its XY translation is exactly zero. Keep the solver's orientation and Z
  // result, but do not let repeated structures or residual IMU velocity walk
  // the scan chain away from its last trustworthy position. The wheel sample
  // is scalar proprioception only; no VKS/GNSS pose enters this correction.
  bool wheel_stationary_translation_held = false;
  Eigen::Matrix4f track_prior = this->T_prior;
  if (wheel_stationary_hold && candidate_pose_valid &&
      matrixFinite(final_candidate) && this->last_gicp_valid_) {
    const Eigen::Vector2f held_xy = this->last_gicp_pose_.block<2, 1>(0, 3);
    wheel_stationary_translation_held =
        (final_candidate.block<2, 1>(0, 3) - held_xy).norm() > 1e-6F;
    final_candidate.block<2, 1>(0, 3) = held_xy;
    track_prior.block<2, 1>(0, 3) = held_xy;
  }

  const bool track_gate_active =
      this->track_constraint_enabled_ && this->track_constraint_ &&
      static_cast<int>(
          this->accepted_gicp_count_.load(std::memory_order_relaxed)) >=
          this->gicp_planar_warmup_accepted_scans_;
  const double track_time_on_track_s =
      this->track_time_on_track_s_.load(std::memory_order_acquire);
  const int track_tuning_index = trackTimeTuningIndex(
      track_time_on_track_s, this->track_time_breakpoints_s_);
  this->track_time_tuning_index_.store(track_tuning_index,
                                       std::memory_order_release);
  TtlTrackDecision track_decision;
  track_decision.accepted = !track_gate_active;
  track_decision.reason = track_gate_active ? TtlTrackRejectReason::kNoLines
                                            : TtlTrackRejectReason::kNone;
  const double base_track_lateral_limit_m = speedAdaptiveLateralLimit(
      this->track_adaptive_lateral_enabled_, wheel_speed_fresh, wheel_speed_mps,
      scan_dt, this->track_lateral_fallback_limit_m_,
      this->track_adaptive_lateral_base_m_,
      this->track_adaptive_lateral_speed_distance_ratio_,
      this->track_adaptive_lateral_max_m_,
      this->track_adaptive_lateral_max_scan_dt_s_);
  const double track_lateral_limit_m = trackTimeTunedValue(
      base_track_lateral_limit_m, track_time_on_track_s,
      this->track_time_breakpoints_s_, this->track_time_max_applied_lateral_m_);
  const double track_allowed_along_correction_m = trackTimeTunedValue(
      this->track_max_along_correction_m_, track_time_on_track_s,
      this->track_time_breakpoints_s_,
      this->track_time_max_along_corrections_m_);
  TrackHessianInformation track_hessian_information;
  const double configured_track_along_gain = trackTimeTunedValue(
      this->track_along_correction_gain_, track_time_on_track_s,
      this->track_time_breakpoints_s_, this->track_time_along_gains_);
  double track_along_gain = configured_track_along_gain;
  const bool track_along_recovery_attempt =
      solve_limits.recovery_attempt && this->previous_failure_track_along_ &&
      this->track_recovery_along_max_raw_correction_m_ > 0.0 &&
      this->track_recovery_along_gain_ > 0.0 &&
      this->track_recovery_along_max_applied_correction_m_ > 0.0;
  if (track_gate_active && candidate_pose_valid &&
      matrixFinite(final_candidate)) {
    const TtlTrackDecision raw_track_decision =
        this->track_constraint_->evaluate(track_prior, final_candidate);
    if (raw_track_decision.candidate.valid &&
        std::isfinite(raw_track_decision.candidate.heading_rad) &&
        analysis_hessian.allFinite() && matrixFinite(this->T_prior)) {
      track_hessian_information = trackDirectionalHessianInformation(
          analysis_hessian, this->T_prior.block<3, 1>(0, 3).cast<double>(),
          raw_track_decision.candidate.heading_rad);
    }
    track_along_gain = informationAdaptiveAlongGain(
        this->track_adaptive_along_enabled_,
        track_hessian_information.tangent_marginal_stiffness,
        num_correspondences, this->track_adaptive_along_low_info_per_corr_,
        this->track_adaptive_along_full_info_per_corr_,
        this->track_adaptive_along_min_gain_, configured_track_along_gain);
    if (track_along_recovery_attempt) {
      // A normal frame still uses the configured information-adaptive gain,
      // including gain=0 for weak tangent geometry. Only the frame directly
      // following a TTL along-limit rejection receives a bounded correction
      // opportunity. This lets LiDAR close a growing wheel/IMU propagation
      // bias without turning the TTL tangent into a continuously open DoF.
      track_along_gain =
          std::max(track_along_gain, this->track_recovery_along_gain_);
    }
    track_decision = this->track_constraint_->constrainTranslation(
        track_prior, &final_candidate, track_lateral_limit_m, track_along_gain,
        track_along_recovery_attempt
            ? this->track_recovery_along_max_raw_correction_m_
            : track_allowed_along_correction_m,
        track_along_recovery_attempt
            ? this->track_recovery_along_max_applied_correction_m_
            : std::numeric_limits<double>::quiet_NaN());
  }
  const bool track_along_scaled = track_decision.along_scaled;
  const bool track_lateral_clamped = track_decision.lateral_clamped;
  const bool track_translation_modified = track_along_scaled ||
                                          track_lateral_clamped ||
                                          wheel_stationary_translation_held;

  // [REVIEW FIX 2026-07-08 P1] When the pose being APPLIED differs from the
  // optimizer solution (degeneracy/yaw projection or TTL-tangent scaling),
  // re-validate fitness and correspondence support AT final_candidate. The
  // raw optimizer fitness describes a pose nobody is applying: a wrong-basin
  // solution with plausible raw fitness could have its yaw vetoed and still
  // land its translation unvalidated. One extra kd-tree/linearize pass, and
  // only on modified (degenerate/vetoed) frames. If the modified pose cannot
  // be evaluated at all, fail safe: force the fitness gate to reject.
  int support_corr = num_correspondences;
  double support_ratio = correspondence_ratio;
  bool fitness_reevaluated = false;
  auto rescore_applied_pose = [&]() {
    const Eigen::Matrix4f T_eval =
        this->scan_in_world_frame_
            ? Eigen::Matrix4f(final_candidate * this->T_prior.inverse())
            : Eigen::Matrix4f(final_candidate * T_base_lidar);
    double fit_applied = std::numeric_limits<double>::quiet_NaN();
    int corr_applied = 0;
    fitness_reevaluated = true;
    if (this->gicp.evaluateFitnessAt(T_eval, &fit_applied, &corr_applied)) {
      fitness_score = fit_applied;
      this->last_fitness_score_ = fitness_score;
      support_corr = corr_applied;
      support_ratio =
          this->current_scan->points.empty()
              ? 0.0
              : static_cast<double>(corr_applied) /
                    static_cast<double>(this->current_scan->points.size());
      if (fitness_baseline > 1e-9 && std::isfinite(fitness_score)) {
        fitness_ratio = fitness_score / fitness_baseline;
      }
    } else {
      fitness_score = std::numeric_limits<double>::infinity();
      this->last_fitness_score_ = fitness_score;
      support_corr = 0;
      support_ratio = 0.0;
    }
  };
  if (candidate_pose_valid &&
      ((degen.valid && degen.modified) || track_translation_modified) &&
      matrixFinite(final_candidate)) {
    rescore_applied_pose();
  }

  // [P2 FIX 2026-07-10g] Revisit the SOFT yaw veto with the APPLIED-pose
  // fitness ratio. Soft arming was decided from the RAW optimizer ratio
  // BEFORE the projection; the rescore above can raise the ratio past
  // yawGate/fitnessRatio — i.e. the pose actually being applied is now a
  // low-confidence match — while its yaw correction sits between maxCorrDeg
  // (soft, 1.5) and hardMaxCorrDeg (8), a band no other gate strips. Apply
  // the same veto the projector would have: zero the world-tangent yaw
  // component of the delta vs T_prior (keep the IMU yaw), leave translation
  // untouched, then re-score once more so every downstream gate judges the
  // pose that is actually applied. Hard arming needs no revisit (it is
  // unconditional and already ran in the projector).
  if (fitness_reevaluated && candidate_pose_valid && !yaw_soft_armed &&
      this->yaw_gate_enable_ && this->yaw_gate_max_corr_deg_ > 0.0 &&
      fitness_ratio > 0.0 && fitness_ratio > this->yaw_gate_fitness_ratio_ &&
      matrixFinite(final_candidate)) {
    const double yaw_innov_applied =
        yawInnovationDeg(this->T_prior, final_candidate);
    if (std::abs(yaw_innov_applied) > this->yaw_gate_max_corr_deg_) {
      const Eigen::Matrix3d R_prior_d =
          this->T_prior.block<3, 3>(0, 0).cast<double>();
      const Eigen::Matrix3d R_fc =
          final_candidate.block<3, 3>(0, 0).cast<double>();
      const Eigen::AngleAxisd aa(R_fc * R_prior_d.transpose());
      Eigen::Vector3d omega_d = aa.angle() * aa.axis();
      omega_d.z() = 0.0;
      Eigen::Matrix3d R_delta = Eigen::Matrix3d::Identity();
      const double ang = omega_d.norm();
      if (ang > 1e-12) {
        R_delta = Eigen::AngleAxisd(ang, omega_d / ang).toRotationMatrix();
      }
      final_candidate.block<3, 3>(0, 0) = (R_delta * R_prior_d).cast<float>();
      degen.yaw_vetoed = true;
      degen.modified = true;
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Soft yaw veto (post-rescore): applied-pose ratio %.2f > %.2f armed "
          "the "
          "gate; %.2f deg yaw correction stripped (IMU yaw kept)",
          fitness_ratio, this->yaw_gate_fitness_ratio_, yaw_innov_applied);
      if (matrixFinite(final_candidate)) {
        rescore_applied_pose(); // gates must judge the yaw-stripped pose
      } else {
        fitness_score = std::numeric_limits<double>::infinity(); // fail closed
        this->last_fitness_score_ = fitness_score;
      }
    }
  }

  double guess_to_solution_trans = -1.0;
  double guess_to_solution_rot_deg = -1.0;
  if (candidate_pose_valid) {
    guess_to_solution_trans =
        deltaTranslationNorm(guess_pose_map, candidate_pose);
    guess_to_solution_rot_deg =
        rotationDistanceDeg(guess_pose_map, candidate_pose);
  }

  double imu_buffer_span = -1.0;
  double scan_to_latest_imu_lag = -1.0;
  {
    std::lock_guard<std::mutex> imu_lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      const double latest_imu_stamp = this->imu_buffer.front().stamp;
      const double oldest_imu_stamp = this->imu_buffer.back().stamp;
      imu_buffer_span = latest_imu_stamp - oldest_imu_stamp;
      scan_to_latest_imu_lag = this->scan_stamp.seconds() - latest_imu_stamp;
    }
  }

  // Jump is measured against the IMU-predicted prior, not the last GICP pose.
  // This asks "did GICP disagree with IMU?" (catastrophic failure indicator)
  // instead of "did the vehicle move far?" (expected at highway speed + time
  // gaps).
  double jump_trans = -1.0;
  double jump_rot_deg = -1.0;
  if (candidate_pose_valid) {
    jump_trans = deltaTranslationNorm(this->T_prior, candidate_pose);
    jump_rot_deg = rotationDistanceDeg(this->T_prior, candidate_pose);
  }
  // Speed/scan_dt-aware jump thresholds (P2#2). The jump is GICP-vs-IMU-prior,
  // so the legitimate disagreement scales with how far the prior could have
  // drifted: ~speed*scan_dt for translation and ~scan_dt for rotation (scan_dt
  // here is the time since the last accepted pose, so it grows during a failure
  // streak). This loosens the gate after a gap / at high speed so a valid
  // reacquisition is not rejected, while keeping it tight for slow, short-gap
  // scans. Setting the scales to 0 reproduces the fixed thresholds.
  const double scan_dt_clamped = std::min(std::max(scan_dt, 0.0), 1.0);
  const double eff_jump_trans_m = pre_solve_gate.max_translation_m;
  const double eff_jump_rot_deg = pre_solve_gate.max_rotation_deg;
  const bool large_jump =
      candidate_pose_valid &&
      (jump_trans > eff_jump_trans_m || jump_rot_deg > eff_jump_rot_deg);
  // The jump GATE evaluates the pose that would actually be applied. A partial
  // update can only shrink the GICP-vs-prior delta, so this is never looser
  // than the raw-candidate check; raw jump_trans/jump_rot_deg stay in the
  // debug topics/log as the unprojected disagreement signal.
  double final_jump_trans = jump_trans;
  double final_jump_rot_deg = jump_rot_deg;
  if (candidate_pose_valid &&
      ((degen.valid && degen.modified) || track_translation_modified)) {
    final_jump_trans = deltaTranslationNorm(this->T_prior, final_candidate);
    final_jump_rot_deg = rotationDistanceDeg(this->T_prior, final_candidate);
  }
  const bool large_jump_final =
      candidate_pose_valid && (final_jump_trans > eff_jump_trans_m ||
                               final_jump_rot_deg > eff_jump_rot_deg);

  // P1 yaw-safety fix #2: yaw split out of the 3D rotation jump gate. The
  // generic gate (30 deg + 60 deg/s * scan_dt) exists to admit re-acquisition
  // after gaps, but for YAW that envelope is physically absurd on a ground
  // vehicle — the IMU-integrated prior cannot be tens of degrees wrong over a
  // 0.1-0.3 s gap (runs 19/20: 45-47 deg proposals passed the ~36-45 deg
  // effective gate and produced 78-98 deg heading errors). Yaw innovation vs
  // T_prior gets its own tight budget with a hard absolute cap that scan_dt
  // scaling can never raise into relocalization territory:
  //   eff_yaw = min(yaw_max_deg + yaw_dt_scale_deg * scan_dt,
  //   yaw_total_max_deg)
  // Evaluated on the APPLIED pose (post yaw-veto/projection), so a vetoed
  // frame — whose yaw already equals the IMU prior — passes and keeps its
  // translation information. Controlled relocalization stays available via
  // GT snap recovery, which bypasses scan gates by design.
  const double yaw_innov_raw_deg =
      candidate_pose_valid ? yawInnovationDeg(this->T_prior, candidate_pose)
                           : -1.0;
  const double yaw_innov_final_deg =
      candidate_pose_valid
          ? (((degen.valid && degen.modified) || track_translation_modified)
                 ? yawInnovationDeg(this->T_prior, final_candidate)
                 : yaw_innov_raw_deg)
          : -1.0;
  const double eff_yaw_max_deg =
      (this->jump_yaw_max_deg_ > 0.0)
          ? std::min(this->jump_yaw_max_deg_ +
                         this->jump_yaw_dt_scale_deg_ * scan_dt_clamped,
                     this->jump_yaw_total_max_deg_ > 0.0
                         ? this->jump_yaw_total_max_deg_
                         : std::numeric_limits<double>::infinity())
          : std::numeric_limits<double>::infinity();
  const bool yaw_jump_final = candidate_pose_valid &&
                              std::isfinite(eff_yaw_max_deg) &&
                              yaw_innov_final_deg > eff_yaw_max_deg;

  // PR#6: BOUNDED non-converged fitness fallback. The "effectively converged"
  // rule (non-converged but fitness under the absolute threshold) exists for
  // the max-iterations-at-highway-speed case, where the correction is small.
  // Run5-on-Run3 showed it also admitting wrong-basin results with corrections
  // of 3.27 m/11.2 deg and 9.21 m/10.7 deg — and each accept reset the
  // consecutive-failure counter, locking GT recovery out until the pose had
  // drifted far from INS. Keep the fallback only for SMALL corrections
  // (vs the IMU prior, on the pose actually applied); larger non-converged
  // candidates are classified failed_to_converge. <=0 disables either bound.
  // PR-validated: nonconv accepts 1145->397, accepts with INS err >=50 m 99->3.
  const bool nonconv_fallback_ok =
      candidate_pose_valid &&
      fitness_score <= this->gicp_fitness_reject_threshold_ &&
      (this->gicp_nonconv_ok_max_trans_m_ <= 0.0 ||
       final_jump_trans <= this->gicp_nonconv_ok_max_trans_m_) &&
      (this->gicp_nonconv_ok_max_rot_deg_ <= 0.0 ||
       final_jump_rot_deg <= this->gicp_nonconv_ok_max_rot_deg_);
  const bool effectively_converged = converged || nonconv_fallback_ok;

  const bool track_candidate_allowed =
      !track_gate_active || track_decision.accepted;

  auto build_scan_debug_log = [&](const char *status) {
    const double pose_stamp = (this->t_prior_stamp_ > 0.0)
                                  ? this->t_prior_stamp_
                                  : this->scan_stamp.seconds();
    std::ostringstream oss;
    oss << std::fixed << "SCAN DEBUG | status=" << status
        << " stamp=" << std::setprecision(3) << this->scan_stamp.seconds()
        << " pose_stamp=" << std::setprecision(6) << pose_stamp
        << " input_frame=" << this->last_scan_input_frame_
        << " raw=" << this->last_raw_point_count_
        << " pre=" << this->last_preprocessed_point_count_ << " guess={"
        << poseSummary(guess_pose_map) << "}" << " pos_seed=["
        << (gicp_position_seed_applied ? 1 : 0)
        << ",step=" << scalarSummary(gicp_position_seed_step_m) << "m]"
        << " guess_from_last=[" << scalarSummary(guess_from_last_trans) << "m,"
        << scalarSummary(guess_from_last_rot_deg)
        << "deg,dz=" << scalarSummary(guess_from_last_vertical) << "m]"
        << " gicp_ms=" << scalarSummary(elapsed_ms, 2)
        << " iterations=" << gicp_iterations << " inner_trials=["
        << gicp_inner_trials << ",rejected=" << gicp_rejected_inner_trials
        << "]" << " solve_limit=[" << scalarSummary(active_gicp_budget_ms, 1)
        << "ms," << active_gicp_max_iterations
        << "iter,recovery=" << (solve_limits.recovery_attempt ? 1 : 0)
        << ",overload_gap=" << (this->last_scan_followed_overload_ ? 1 : 0)
        << ",strict_resync_gap="
        << (this->last_scan_followed_strict_resync_ ? 1 : 0)
        << ",scan_gap=" << (scan_dt > 1.0 ? 1 : 0)
        << ",expanded=" << (solve_limits.expanded ? 1 : 0) << "]"
        << " gicp_stage_ms=[tree=" << scalarSummary(source_tree_ms, 2)
        << ",cov=" << scalarSummary(source_covariance_ms, 2)
        << ",target_cov=" << scalarSummary(target_covariance_ms, 2)
        << ",optimizer=" << scalarSummary(optimizer_ms, 2)
        << ",timeout=" << timeout_stage << "]" << " local_map=[active="
        << (this->local_map_target_active_ ? "local" : "full")
        << ",points=" << this->local_map_current_target_points_
        << ",build_ms=" << scalarSummary(this->local_map_last_build_ms_, 1)
        << ",builds=" << this->local_map_build_count_
        << ",fallbacks=" << this->local_map_global_fallback_count_ << "]"
        << " pipeline_stage_ms=[deskew="
        << scalarSummary(this->last_deskew_ms_, 2)
        << ",preprocess=" << scalarSummary(this->last_preprocess_ms_, 2) << "]"
        << " deskew_detail_ms=[setup="
        << scalarSummary(this->last_deskew_setup_ms_, 2)
        << ",imu_wait=" << scalarSummary(this->last_deskew_imu_wait_ms_, 2)
        << ",imu_integrate="
        << scalarSummary(this->last_deskew_imu_integrate_ms_, 2)
        << ",transform=" << scalarSummary(this->last_deskew_transform_ms_, 2)
        << "]" << " scan_total_ms="
        << scalarSummary(std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() -
                             this->scan_pipeline_start_)
                             .count(),
                         2)
        << " converged=" << (converged ? "true" : "false")
        << " timed_out=" << (gicp_timed_out ? 1 : 0)
        << " budget_limited=" << (gicp_budget_limited ? 1 : 0)
        << " fitness=" << scalarSummary(fitness_score, 6)
        << " fit_ratio=" << scalarSummary(fitness_ratio, 3) << " degen=[r"
        << degen.degen_rot_axes << ",t" << degen.degen_trans_axes
        << ",yaw_veto=" << (degen.yaw_vetoed ? 1 : 0)
        << ",rp_clamp=" << (degen.rp_clamped ? 1 : 0)
        << ",partial=" << ((degen.valid && degen.modified) ? 1 : 0)
        << ",track_along=" << (track_along_scaled ? 1 : 0)
        << ",track_lateral=" << (track_lateral_clamped ? 1 : 0) << "]"
        << " final_error=" << scalarSummary(final_error, 6)
        << " correspondences=" << num_correspondences << "/"
        << this->current_scan->points.size()
        << " ratio=" << scalarSummary(correspondence_ratio, 3) << " support=["
        << support_corr << "," << scalarSummary(support_ratio, 3)
        << ",reeval=" << (fitness_reevaluated ? 1 : 0) << "]"
        << " guess_to_solution=[" << scalarSummary(guess_to_solution_trans)
        << "m," << scalarSummary(guess_to_solution_rot_deg) << "deg]"
        << " jump=[" << scalarSummary(jump_trans) << "m,"
        << scalarSummary(jump_rot_deg) << "deg]" << " yaw_innov=["
        << scalarSummary(yaw_innov_raw_deg, 2)
        << "deg,fin=" << scalarSummary(yaw_innov_final_deg, 2) << "deg]"
        << " track_stiff=[t="
        << scalarSummary(track_hessian_information.tangent_marginal_stiffness,
                         1)
        << ",n="
        << scalarSummary(track_hessian_information.normal_marginal_stiffness, 1)
        << "]" << " ins_dyaw=" << scalarSummary(this->last_ins_yaw_diff_deg_, 2)
        << "deg" << " track=[active=" << (track_gate_active ? 1 : 0)
        << ",pass=" << (track_candidate_allowed ? 1 : 0)
        << ",reason=" << ttlTrackRejectReasonName(track_decision.reason)
        << ",line=" << track_decision.candidate.line_name << ",corridor_excess="
        << scalarSummary(track_decision.corridor_excess_m)
        << "m,ds=" << scalarSummary(track_decision.along_correction_m)
        << "m,raw_ds=" << scalarSummary(track_decision.raw_along_correction_m)
        << "m,along_gain=" << scalarSummary(track_along_gain, 3)
        << ",along_recovery=" << (track_along_recovery_attempt ? 1 : 0)
        << ",along_apply_limit="
        << scalarSummary(track_decision.applied_along_limit_m) << "m"
        << ",along_scaled=" << (track_decision.along_scaled ? 1 : 0)
        << ",dl=" << scalarSummary(track_decision.lateral_correction_m)
        << "m,raw_dl=" << scalarSummary(track_decision.raw_lateral_correction_m)
        << "m,lateral_clamped=" << (track_decision.lateral_clamped ? 1 : 0)
        << ",lat_limit=" << scalarSummary(track_lateral_limit_m) << "m"
        << ",z_target=" << scalarSummary(track_z_target_m) << "m]"
        << " imu_buffer_span=" << scalarSummary(imu_buffer_span) << "s"
        << " scan_to_latest_imu_lag=" << scalarSummary(scan_to_latest_imu_lag)
        << "s" << " wheel_speed=[" << scalarSummary(wheel_speed_mps, 3)
        << "mps,age=" << scalarSummary(wheel_speed_age_s, 3)
        << "s,used=" << (wheel_speed_fresh ? 1 : 0)
        << ",stationary_hold=" << (wheel_stationary_hold ? 1 : 0) << "]"
        << " gt_dropped_invalid=" << this->gt_dropped_invalid_.load()
        << " gt_dropped_frame=" << this->gt_dropped_frame_.load() << " concat=["
        << this->concat_last_merged_aux_ << "/" << this->aux_lidars_.size();
    for (size_t i = 0; i < this->concat_last_aux_dt_.size(); ++i) {
      oss << ",dt" << i << "=" << scalarSummary(this->concat_last_aux_dt_[i], 3)
          << "s,pts" << i << "=" << this->concat_last_aux_points_[i];
    }
    oss << ",span=" << scalarSummary(this->last_scan_time_span_s_, 3) << "s]"
        << " hessian_cond=" << scalarSummary(hessian_condition, 3)
        << " candidate={" << poseSummary(candidate_pose) << "}";

    if (this->last_gicp_valid_) {
      oss << " last_good={" << poseSummary(this->last_gicp_pose_) << "}";
    }
    return oss.str();
  };

  // effectively_converged is computed above (PR#6 bounded fallback) so the
  // debug `converged` topic publishes the same decision the gate uses.

  const RegistrationGateConfig gate_config{
      this->gicp_min_correspondences_,     this->gicp_min_corr_ratio_,
      this->gicp_fitness_reject_threshold_, this->fitness_ratio_reject_,
      this->degen_partial_update_enable_,  this->gicp_reject_large_jumps_,
      this->gicp_hessian_cond_max_,        this->gicp_hessian_fitness_warn_,
      this->gicp_hessian_trans_warn_m_,
      this->gicp_hessian_rot_warn_deg_};
  const RegistrationGateInput gate_input{effectively_converged,
                                         candidate_pose_valid,
                                         analysis_hessian.allFinite(),
                                         support_corr,
                                         support_ratio,
                                         fitness_score,
                                         yaw_jump_final,
                                         fitness_ratio,
                                         eigen_projection_wanted,
                                         degen.valid,
                                         degen.fully_degenerate,
                                         large_jump_final,
                                         hessian_condition,
                                         guess_to_solution_trans,
                                         guess_to_solution_rot_deg};
  const RegistrationGateDecision gate =
      evaluateRegistrationGate(gate_config, gate_input);

  const bool gicp_rejected_fitness = gate.rejected_fitness;
  const bool gicp_rejected_fitness_ratio = gate.rejected_fitness_ratio;
  const bool gicp_rejected_jump = gate.rejected_jump;
  const bool gicp_rejected_yaw = gate.rejected_yaw;
  const bool gicp_rejected_hessian = gate.rejected_hessian;
  const bool gicp_rejected_support = gate.rejected_support;
  const bool gicp_rejected_track = gate.accepted && !track_candidate_allowed;
  const bool gicp_accepted = gate.accepted && track_candidate_allowed;
  const bool gicp_partial = gicp_accepted && ((degen.valid && degen.modified) ||
                                              track_translation_modified);
  if (pre_solve_guess_rejected) {
    RCLCPP_WARN(
        this->get_logger(),
        "GICP SKIPPED (prior jump %.2fm/%.2fdeg, dz=%.2fm exceeds pre-solve "
        "envelope %.2fm/%.2fdeg, dz<=%.2fm): %s",
        guess_from_last_trans, guess_from_last_rot_deg,
        guess_from_last_vertical, pre_solve_gate.max_translation_m,
        pre_solve_gate.max_rotation_deg, pre_solve_gate.max_vertical_m,
        build_scan_debug_log("rejected_pre_solve_guess").c_str());
  } else if (!candidate_pose_valid) {
    RCLCPP_WARN(this->get_logger(), "%s",
                build_scan_debug_log("invalid_solution").c_str());
  } else if (gicp_timed_out) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (runtime %.1fms reached %s cutoff %.1fms; "
                "finalization reserve %.1fms): %s",
                elapsed_ms,
                hard_runtime_exceeded ? "absolute result-age" : "cooperative",
                hard_runtime_exceeded ? this->gicp_hard_result_max_age_ms_
                                      : active_gicp_budget_ms,
                hard_runtime_exceeded ? 0.0
                                      : this->gicp_finalization_reserve_ms_,
                build_scan_debug_log("rejected_timeout").c_str());
  } else if (!effectively_converged) {
    RCLCPP_WARN(this->get_logger(), "%s",
                build_scan_debug_log("failed_to_converge").c_str());
  } else if (gicp_rejected_track) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (TTL track constraint: %s, line=%s, "
                "corridor_excess=%.2fm, "
                "along=%.2fm, lateral=%.2fm): %s",
                ttlTrackRejectReasonName(track_decision.reason),
                track_decision.candidate.line_name.c_str(),
                track_decision.corridor_excess_m,
                track_decision.raw_along_correction_m,
                track_decision.lateral_correction_m,
                build_scan_debug_log("rejected_track_constraint").c_str());
  } else if (gicp_rejected_fitness) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (fitness=%.4f > threshold=%.4f): %s",
                fitness_score, this->gicp_fitness_reject_threshold_,
                build_scan_debug_log("rejected_fitness").c_str());
  } else if (gicp_rejected_support) {
    RCLCPP_WARN(
        this->get_logger(),
        "GICP REJECTED (support: corr=%d ratio=%.3f below min [%d, %.3f]): %s",
        support_corr, support_ratio, this->gicp_min_correspondences_,
        this->gicp_min_corr_ratio_,
        build_scan_debug_log("rejected_support").c_str());
  } else if (gicp_rejected_yaw) {
    RCLCPP_WARN(
        this->get_logger(),
        "GICP REJECTED (yaw innovation %.2f deg > eff %.2f deg vs IMU prior @ "
        "scan_dt=%.3fs — "
        "physically impossible heading correction for a ground vehicle): %s",
        yaw_innov_final_deg, eff_yaw_max_deg, scan_dt_clamped,
        build_scan_debug_log("rejected_yaw").c_str());
  } else if (gicp_rejected_fitness_ratio) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (fitness_ratio=%.3f > %.3f, baseline=%.4f — "
                "wrong-basin signature): %s",
                fitness_ratio, this->fitness_ratio_reject_, fitness_baseline,
                build_scan_debug_log("rejected_fitness_ratio").c_str());
  } else if (gicp_rejected_hessian) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (hessian_cond=%.3e > %.3e AND "
                "[fitness=%.4f|trans=%.3fm|rot=%.3fdeg] crossed "
                "[%.4f|%.3fm|%.3fdeg] — degenerate slide): %s",
                hessian_condition, this->gicp_hessian_cond_max_, fitness_score,
                guess_to_solution_trans, guess_to_solution_rot_deg,
                this->gicp_hessian_fitness_warn_,
                this->gicp_hessian_trans_warn_m_,
                this->gicp_hessian_rot_warn_deg_,
                build_scan_debug_log("rejected_hessian").c_str());
  } else if (gicp_rejected_jump) {
    RCLCPP_WARN(this->get_logger(),
                "GICP REJECTED (jump dT=%.3fm dR=%.2fdeg > eff thresholds "
                "[%.2fm, %.2fdeg] @ speed=%.1fm/s scan_dt=%.3fs): %s",
                final_jump_trans, final_jump_rot_deg, eff_jump_trans_m,
                eff_jump_rot_deg, static_cast<double>(speed_est),
                scan_dt_clamped, build_scan_debug_log("rejected_jump").c_str());
  } else if (large_jump && !gicp_partial) {
    RCLCPP_WARN(this->get_logger(), "%s",
                build_scan_debug_log("large_jump").c_str());
  } else if (this->debug_verbose_scan_log_) {
    // "ok_partial" = accepted after degeneracy projection / yaw veto shrank
    // the correction; grep-compatible with "status=ok" prefix matching is NOT
    // preserved on purpose so audits can split the two populations.
    RCLCPP_INFO(
        this->get_logger(), "%s",
        build_scan_debug_log(gicp_partial ? "ok_partial" : "ok").c_str());
  }

  if (gicp_accepted) {
    // Treat the accepted small_gicp pose as a map measurement at the scan's
    // median acquisition time.  The delayed filter fuses x/y/yaw with its
    // covariance prior and replays all newer IMU/wheel observations before the
    // product is published.  The raw validated candidate remains available on
    // the existing debug topic and in the accepted-pose pair.
    DelayedGicpFusionResult delayed_fusion_result;
    const bool delayed_fusion_applied = this->applyDelayedFusionToAcceptedPose(
        final_candidate, wheel_stationary_hold, &delayed_fusion_result);
    if (delayed_fusion_applied) {
      const auto &measurement_state =
          delayed_fusion_result.measurement_time_state.nominal;
      Eigen::Matrix4f fused_scan_pose = Eigen::Matrix4f::Identity();
      fused_scan_pose.block<3, 3>(0, 0) =
          measurement_state.orientation.cast<float>().toRotationMatrix();
      fused_scan_pose.block<3, 1>(0, 3) =
          measurement_state.position.cast<float>();
      if (this->gicp_dof_mode_ == "planar") {
        fused_scan_pose(2, 3) = final_candidate(2, 3);
      }
      final_candidate = fused_scan_pose;
    }

    // P1: apply the (possibly degeneracy-projected) candidate, and feed the
    // per-map fitness baseline from accepted frames only, so wrong-basin /
    // rejected fitness never inflates the baseline the gates divide by.
    this->current_pose = final_candidate;
    const double accepted_z = static_cast<double>(final_candidate(2, 3));
    this->track_z_hold_m_.store(accepted_z, std::memory_order_relaxed);
    this->track_z_hold_valid_.store(std::isfinite(accepted_z),
                                    std::memory_order_release);
    if (this->track_elevation_enabled_ && this->track_constraint_ &&
        !this->track_z_offset_valid_) {
      const auto elevation =
          this->track_constraint_->projectElevation(final_candidate);
      if (elevation.valid) {
        this->track_z_offset_m_ =
            static_cast<double>(final_candidate(2, 3)) - elevation.z;
        this->track_z_offset_valid_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "Calibrated static TTL elevation datum from accepted LiDAR pose: "
            "map_z - ttl_z = %.3fm (line=%s, center_distance=%.2fm)",
            this->track_z_offset_m_, elevation.line_name.c_str(),
            elevation.center_distance_m);
      }
    }
    if (this->fitness_baseline_enable_ && std::isfinite(fitness_score) &&
        fitness_score >= 0.0) {
      this->fitness_history_.push_back(fitness_score);
      while (static_cast<int>(this->fitness_history_.size()) >
             this->fitness_baseline_window_) {
        this->fitness_history_.pop_front();
      }
    }

    // Update lidar pose for next iteration
    Eigen::Vector3f new_p = this->current_pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotSO3 = this->current_pose.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotSO3);
    q.normalize();

    {
      // [P2 FIX 2026-07-09] Seed writes under the owner lock (see seed_mtx_
      // in the header): a cross-thread reinit (RViz / RTK full seed) must
      // never interleave with these. Lock order here: pose -> seed.
      std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
      const double scan_velocity_dt =
          (this->base_pose_stamp_ > 0.0)
              ? this->t_prior_stamp_ - this->base_pose_stamp_
              : this->observer_dt_;
      this->basePose.p = new_p;
      this->basePose.q = q;
      // [REVIEW FIX 2026-07-08] The accepted candidate is the pose at the
      // median point time of this scan (the cloud was deskewed to
      // frames[median]).
      this->base_pose_stamp_ = this->t_prior_stamp_;
      // Keep the next scan seed independent of optimizer wall time. The IMU
      // prediction and GICP residual are both at this scan's median timestamp.
      // perception-ws assumes its degeneracy remap has already removed the
      // LiDAR-unobservable body-x component. DLIO's remap is conditional, so
      // enforce that observability boundary here before residual/dt feedback;
      // otherwise repeated longitudinal corrections can inflate speed and
      // fling the next scan seed into a parallel track section.
      const Eigen::Vector3f position_residual =
          new_p - this->T_prior.block<3, 1>(0, 3);
      // With a fresh proprioceptive speed, keep scan residual feedback only
      // in LiDAR-observable lateral/vertical directions and let wheels own
      // body-x. If wheels are absent/stale, preserve the historical full
      // residual fallback instead of allowing unbounded inertial speed drift.
      const Eigen::Vector3f feedback_residual =
          wheel_speed_fresh
              ? observablePositionResidual(position_residual,
                                           this->T_prior.block<3, 1>(0, 0))
              : position_residual;
      if (delayed_fusion_applied) {
        this->prev_vel = delayed_fusion_result.measurement_time_state.nominal
                             .velocity.cast<float>();
        if (this->gicp_dof_mode_ == "planar") {
          this->prev_vel.z() = 0.0F;
        }
        this->T_prior_velocity_ = this->prev_vel;
      } else if (wheel_stationary_hold) {
        this->prev_vel.setZero();
        this->T_prior_velocity_.setZero();
      } else {
        this->prev_vel = scanVelocityFeedback(
            this->T_prior_velocity_, feedback_residual, scan_velocity_dt,
            this->scan_velocity_feedback_gain_, this->geo_max_state_speed_);
      }
      if (!delayed_fusion_applied && wheel_speed_fresh &&
          !wheel_stationary_hold) {
        this->prev_vel = blendForwardSpeed(
            this->prev_vel, this->T_prior.block<3, 1>(0, 0), wheel_speed_mps,
            this->wheel_speed_feedback_gain_, this->geo_max_state_speed_);
      }
    }
    // Validate GICP result before using it
    bool gicp_valid = std::isfinite(new_p.x()) && std::isfinite(new_p.y()) &&
                      std::isfinite(new_p.z()) && std::isfinite(q.w()) &&
                      std::isfinite(q.x()) && std::isfinite(q.y()) &&
                      std::isfinite(q.z());

    if (!gicp_valid) {
      RCLCPP_WARN(this->get_logger(), "GICP result contains invalid values, "
                                      "skipping geometric observer update");
    } else {
      // Initialize or update geometric observer
      if (!this->geo.first_opt_done) {
        // First time: initialize state to GICP result.
        // [REVIEW FIX 2026-07-08 P2/P3] Under geo.mtx — IMU callbacks are in a
        // REENTRANT group and propagateState may be mid-flight — and the
        // calibrated IMU biases are PRESERVED: the RTK / stationary
        // calibration paths write state.b.* before the first accepted scan,
        // and unconditionally zeroing them here silently discarded that
        // calibration on non-odom-init setups (use_odom_init=true was immune
        // only because odom init sets first_opt_done, skipping this branch).
        std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
        this->state.p = new_p;
        this->state.q = q;
        this->state.v.lin.w = Eigen::Vector3f::Zero();
        this->state.v.lin.b = Eigen::Vector3f::Zero();
        this->state.v.ang.w = Eigen::Vector3f::Zero();
        this->state.v.ang.b = Eigen::Vector3f::Zero();
        if (!this->imu_calibrated_.load()) {
          this->state.b.accel = Eigen::Vector3f::Zero();
          this->state.b.gyro = Eigen::Vector3f::Zero();
        }

        // Initialize geo tracking
        this->geo.prev_p = this->state.p;
        this->geo.prev_q = this->state.q;
        this->geo.prev_vel = Eigen::Vector3f::Zero();
        ++this->observer_epoch_;
        this->observer_pose_history_.clear();
        this->latency_compensated_output_.valid = false;

        // Mark as initialized
        this->geo.first_opt_done = true;
        ++this->geo.update_seq; // discard in-flight propagateState computations

        RCLCPP_INFO(this->get_logger(),
                    "Geometric observer initialized to pos=[%.2f,%.2f,%.2f]",
                    new_p.x(), new_p.y(), new_p.z());
      } else if (!delayed_fusion_applied) {
        // Update geometric observer with GICP measurement (skip on first scan)
        this->updateState();
      }
      if (wheel_stationary_hold && !delayed_fusion_applied) {
        std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
        this->state.p.x() = new_p.x();
        this->state.p.y() = new_p.y();
        this->state.v.lin.w.setZero();
        this->state.v.lin.b.setZero();
        this->geo.prev_p.x() = new_p.x();
        this->geo.prev_p.y() = new_p.y();
        this->geo.prev_vel.setZero();
        ++this->observer_epoch_;
        this->observer_pose_history_.clear();
        // updateState() snapshots the bounded IMU bridge before this gate is
        // applied. Replace that snapshot with the exact accepted pose so
        // residual inertial velocity cannot move a stationary product output
        // a few centimetres backwards.
        auto &output = this->latency_compensated_output_;
        if (output.valid &&
            std::abs(output.measurement_stamp - this->t_prior_stamp_) < 1e-4) {
          output.stationary = true;
          output.measurement_p = new_p;
          output.measurement_q = q;
          if (!stationaryCompensationTarget(output.measurement_p,
                                            output.measurement_q, &output.p,
                                            &output.q)) {
            output.valid = false;
          }
          output.v_lin_body.setZero();
          output.v_ang_body.setZero();
        }
        ++this->geo.update_seq;
      }
    }

    if (this->debug_jump_log_enabled_ && gicp_valid && this->last_gicp_valid_) {
      if (large_jump) {
        const Eigen::Vector3f t_prior = this->T_prior.block<3, 1>(0, 3);
        const Eigen::Vector3f t_corr = optimizer_solution.block<3, 1>(0, 3);
        RCLCPP_WARN(
            this->get_logger(),
            "JUMP DETECTED: dT=%.3fm dR=%.2fdeg | dt=%.3fs fitness=%.6f | "
            "prior=[%.2f,%.2f,%.2f] corr=[%.2f,%.2f,%.2f]",
            jump_trans, jump_rot_deg, scan_dt, fitness_score, t_prior.x(),
            t_prior.y(), t_prior.z(), t_corr.x(), t_corr.y(), t_corr.z());
      }
    }

    // Update last GICP pose after computing jump metrics
    if (gicp_valid) {
      ++this->accepted_gicp_count_;
      this->last_gicp_pose_ = this->current_pose;
      this->last_gicp_stamp_ = this->scan_stamp;
      this->last_gicp_valid_ = true;
      // [REVIEW FIX 2026-07-08 P2] Record the ACCEPTED fitness here (and only
      // here) so the IMU-rate odometry covariance reflects the last accepted
      // scan's quality — not the latest attempt, which may be a rejection
      // with an inflated/+inf score.
      if (std::isfinite(fitness_score) && fitness_score >= 0.0) {
        this->last_accepted_fitness_score_ = fitness_score;
      }
    }

    // Reset consecutive-failure counter on any accepted scan so the GT-recovery
    // trigger only fires on sustained losing streaks.
    this->consecutive_failures_ = 0;
    this->failure_streak_had_timeout_ = false;
    this->previous_failure_track_along_ = false;
    // Mark this as the last known-good fix; the dead-reckoning covariance
    // growth (P3) measures elapsed time from here.
    this->last_accepted_scan_stamp_ = this->scan_stamp.seconds();

    // Accepted scan poses arrive at LiDAR rate.  Keep this detailed success
    // trace behind the explicit per-scan debug switch; lifecycle, map-load,
    // warning, and error messages remain visible in the normal profile.
    if (this->debug_verbose_scan_log_) {
      const Eigen::Vector3f t_corr = optimizer_solution.block<3, 1>(0, 3);
      RCLCPP_INFO(
          this->get_logger(),
          "Localization: ✓ %s%s | fitness=%.6f | time=%.2fms | "
          "correction=[%.3f, %.3f, %.3f] | pose=[%.2f, %.2f, %.2f]",
          converged ? "CONVERGED" : "ACCEPTED(fitness-ok)",
          gicp_partial ? " [PARTIAL: degenerate axes kept on IMU prior]" : "",
          fitness_score, elapsed_ms, t_corr.x(), t_corr.y(), t_corr.z(),
          // [P3 FIX 2026-07-14] Log the local new_p (== basePose.p just
          // written under seed_mtx_) instead of re-reading this->basePose.p
          // unlocked, which raced a cross-thread /initialpose reinit.
          new_p.x(), new_p.y(), new_p.z());
    }
  } else {
    // Any non-accepted scan (failed_to_converge, rejected_fitness,
    // rejected_jump, invalid_solution) falls back to the IMU-integrated prior.
    // Freezing at last_gicp_pose_ causes cascade divergence at feature-poor
    // corners: each subsequent scan's guess drifts further from reality,
    // fitness gets worse, and the optimizer never recovers.
    ++this->consecutive_failures_;
    this->failure_streak_had_timeout_ =
        this->failure_streak_had_timeout_ || gicp_timed_out;
    this->previous_failure_track_along_ =
        gicp_rejected_track &&
        track_decision.reason == TtlTrackRejectReason::kAlongCorrection;
    const char *reason =
        pre_solve_guess_rejected ? "pre-solve prior jump rejected"
        : !candidate_pose_valid  ? "invalid solution"
        : gicp_timed_out         ? "optimization timed out"
        : !effectively_converged ? "failed to converge"
        : gicp_rejected_support ? "insufficient correspondence support"
        : gicp_rejected_track   ? "TTL track constraint rejected"
        : gicp_rejected_fitness ? "fitness rejected"
        : gicp_rejected_yaw     ? "yaw-innovation rejected (impossible heading)"
        : gicp_rejected_fitness_ratio ? "fitness-ratio rejected (wrong basin)"
        : gicp_rejected_hessian       ? "degenerate geometry"
                                      : "jump rejected";
    if (matrixFinite(this->T_prior)) {
      Eigen::Matrix4f fallback_pose = this->T_prior;
      if (wheel_stationary_hold && this->last_gicp_valid_) {
        fallback_pose.block<2, 1>(0, 3) =
            this->last_gicp_pose_.block<2, 1>(0, 3);
      }
      TtlTrackProjection fallback_projection;
      const bool fallback_lateral_clamped =
          track_gate_active &&
          this->track_constraint_->constrainLateralEnvelope(
              &fallback_pose, &fallback_projection);
      // constrainLateralEnvelope() operates on the scan-time fallback pose.
      // The IMU-rate odometry publisher, however, reads state.p directly.
      // Keeping only basePose/current_pose inside the TTL envelope therefore
      // left the live observer free to continue drifting laterally until the
      // next accepted scan (or a GT recovery snap).  Preserve the propagation
      // since the scan by applying the same pure-translation correction to the
      // live state below.
      const Eigen::Vector2f fallback_xy_correction =
          fallback_pose.block<2, 1>(0, 3) - this->T_prior.block<2, 1>(0, 3);
      Eigen::Vector2f fallback_outward = Eigen::Vector2f::Zero();
      const bool fallback_outward_valid =
          fallback_lateral_clamped &&
          fallback_projection.center_distance_m > 1e-9;
      if (fallback_outward_valid) {
        fallback_outward = Eigen::Vector2f(
            static_cast<float>((this->T_prior(0, 3) - fallback_projection.x) /
                               fallback_projection.center_distance_m),
            static_cast<float>((this->T_prior(1, 3) - fallback_projection.y) /
                               fallback_projection.center_distance_m));
      }
      this->current_pose = fallback_pose;
      const Eigen::Vector3f new_p = fallback_pose.block<3, 1>(0, 3);
      Eigen::Quaternionf q(fallback_pose.block<3, 3>(0, 0));
      q.normalize();
      // [P2 FIX 2026-07-09] Seed writes under the owner lock (order:
      // pose -> seed -> geo, consistent with the accept path and deskew).
      {
        std::lock_guard<std::mutex> seed_lock(this->seed_mtx_);
        this->basePose.p = new_p;
        this->basePose.q = q;
        // [REVIEW FIX 2026-07-08] T_prior is also a median-point-time pose.
        this->base_pose_stamp_ = this->t_prior_stamp_;
        // A rejected registration contributes no residual feedback. Continue
        // from the IMU velocity at this exact median timestamp, never from the
        // asynchronous observer sampled after a variable-duration solve.
        if (wheel_stationary_hold) {
          this->prev_vel.setZero();
          this->T_prior_velocity_.setZero();
        } else {
          this->prev_vel = scanVelocityFeedback(
              this->T_prior_velocity_, Eigen::Vector3f::Zero(),
              this->observer_dt_, 0.0, this->geo_max_state_speed_);
        }
        if (fallback_outward_valid) {
          const float outward_speed =
              this->prev_vel.head<2>().dot(fallback_outward);
          if (outward_speed > 0.0f) {
            this->prev_vel.head<2>() -= outward_speed * fallback_outward;
          }
          this->T_prior_velocity_ = this->prev_vel;
        }
      }
      if (wheel_stationary_hold) {
        std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
        this->state.p.x() = new_p.x();
        this->state.p.y() = new_p.y();
        this->state.v.lin.w.setZero();
        this->state.v.lin.b.setZero();
        this->geo.prev_p.x() = new_p.x();
        this->geo.prev_p.y() = new_p.y();
        this->geo.prev_vel.setZero();
        ++this->observer_epoch_;
        this->observer_pose_history_.clear();
        ++this->geo.update_seq;
      } else if (fallback_lateral_clamped) {
        std::lock_guard<std::mutex> geo_lock(this->geo.mtx);
        this->state.p.head<2>() += fallback_xy_correction;
        this->geo.prev_p.head<2>() += fallback_xy_correction;
        if (fallback_outward_valid) {
          const float outward_speed =
              this->state.v.lin.w.head<2>().dot(fallback_outward);
          if (outward_speed > 0.0f) {
            this->state.v.lin.w.head<2>() -= outward_speed * fallback_outward;
            this->state.v.lin.b =
                this->state.q.conjugate() * this->state.v.lin.w;
          }
        }
        this->geo.prev_vel = this->state.v.lin.w;
        ++this->observer_epoch_;
        this->observer_pose_history_.clear();
        ++this->geo.update_seq;
      }
      RCLCPP_WARN(this->get_logger(),
                  "Localization: ⚠ GICP %s — holding IMU dead-reckoning pose "
                  "[%.2f, %.2f, %.2f]%s | fitness=%.4f time=%.2fms",
                  reason, new_p.x(), new_p.y(), new_p.z(),
                  fallback_lateral_clamped ? " [TTL lateral envelope]" : "",
                  fitness_score, elapsed_ms);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Localization: ⚠ GICP %s — holding last accepted pose (no "
                  "valid T_prior) | fitness=%.4f time=%.2fms",
                  reason, fitness_score, elapsed_ms);
    }

    // GT-driven pose recovery: if enabled and the failure streak has hit the
    // configured threshold, snap state.{pose,velocity} to the time-matched GT
    // sample (transformed into base_frame).  The first rejected complete
    // triple after a strict-sync outage is a special case: continuity is
    // already known lost, so re-anchor immediately instead of accumulating two
    // more failed dead-reckoned priors.  This remains recovery-only; healthy
    // scans still never consume GT position.
    const bool force_absolute_snap =
        pre_solve_guess_rejected || gicp_rejected_fitness_ratio ||
        gicp_rejected_yaw || gicp_rejected_jump;
    if (gicp_timed_out) {
      this->dropQueuedFrontsAfterRegistrationTimeout();
    }
    if (wheel_stationary_hold) {
      // A static scan cannot become a position-loss event. Holding the last
      // scan pose is both safer and independent of the external reference.
      this->consecutive_failures_ = 0;
      this->failure_streak_had_timeout_ = false;
      this->previous_failure_track_along_ = false;
    } else if (this->maybeSnapPoseToGT(
                   reason, force_absolute_snap,
                   this->last_scan_followed_strict_resync_)) {
      this->failure_streak_had_timeout_ = false;
    }
  }

  return gicp_accepted;
}
