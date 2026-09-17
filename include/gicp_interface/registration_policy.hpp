#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gicp_localizer {

// Per-frame solver limits selected from estimator state.  Acquisition and a
// non-timeout recovery attempt may use the wider startup limits.  A timeout
// latches the failure streak onto steady-state limits so an old queued scan
// cannot receive more compute after already missing its deadline.
struct RegistrationSolveLimits {
  bool acquisition_phase = false;
  bool recovery_attempt = false;
  bool expanded = false;
  double optimization_time_ms = 0.0;
  int max_iterations = 1;
};

inline RegistrationSolveLimits selectRegistrationSolveLimits(
    uint64_t accepted_scans, int startup_accepted_scans,
    int consecutive_failures, bool failure_streak_had_timeout,
    double steady_optimization_time_ms, double startup_optimization_time_ms,
    int steady_max_iterations, int startup_max_iterations) {
  RegistrationSolveLimits limits;
  limits.acquisition_phase =
      accepted_scans < static_cast<uint64_t>(std::max(startup_accepted_scans, 0));
  limits.recovery_attempt = consecutive_failures > 0;
  limits.expanded = !failure_streak_had_timeout &&
                    (limits.acquisition_phase || limits.recovery_attempt);
  limits.optimization_time_ms =
      limits.expanded && startup_optimization_time_ms > 0.0
          ? startup_optimization_time_ms
          : steady_optimization_time_ms;
  limits.max_iterations =
      limits.expanded && startup_max_iterations > 0
          ? startup_max_iterations
          : steady_max_iterations;
  return limits;
}

struct PreSolveGuessGate {
  bool rejected = false;
  double max_translation_m = 0.0;
  double max_vertical_m = 0.0;
  double max_rotation_deg = 0.0;
};

// Reject an already-impossible motion prior before entering the nearest-
// neighbour backend.  A far-away query can make a no-correspondence KD search
// dramatically more expensive than a normal local registration, while its
// result is guaranteed to fail the same motion envelope after the solve.
inline PreSolveGuessGate evaluatePreSolveGuessGate(
    bool last_pose_valid, double guess_translation_m,
    double guess_vertical_m, double guess_rotation_deg, double speed_mps,
    double scan_dt_s,
    double base_translation_m, double translation_speed_scale,
    double max_vertical_m, double base_rotation_deg,
    double rotation_dt_scale_deg) {
  PreSolveGuessGate gate;
  const double bounded_speed =
      std::isfinite(speed_mps) ? std::max(speed_mps, 0.0) : 0.0;
  const double bounded_dt =
      std::isfinite(scan_dt_s) ? std::clamp(scan_dt_s, 0.0, 1.0) : 0.0;
  gate.max_translation_m =
      base_translation_m + translation_speed_scale * bounded_speed * bounded_dt;
  gate.max_vertical_m = max_vertical_m;
  gate.max_rotation_deg =
      base_rotation_deg + rotation_dt_scale_deg * bounded_dt;

  if (!last_pose_valid || !std::isfinite(guess_translation_m) ||
      !std::isfinite(guess_vertical_m) || !std::isfinite(guess_rotation_deg)) {
    gate.rejected = last_pose_valid;
    return gate;
  }
  const bool translation_rejected =
      base_translation_m > 0.0 && guess_translation_m > gate.max_translation_m;
  // Ground vehicles can legitimately travel tens of metres horizontally
  // during a recovery gap, but not vertically.  Keep this component bound
  // independent of speed so a high horizontal speed cannot admit a large Z
  // error into the nearest-neighbour backend.
  const bool vertical_rejected =
      max_vertical_m > 0.0 && guess_vertical_m > max_vertical_m;
  const bool rotation_rejected =
      base_rotation_deg > 0.0 && guess_rotation_deg > gate.max_rotation_deg;
  gate.rejected = translation_rejected || vertical_rejected || rotation_rejected;
  return gate;
}

}  // namespace gicp_localizer
