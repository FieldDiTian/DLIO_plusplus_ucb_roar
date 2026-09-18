#ifndef GICP_LOCALIZER__TTL_TRACK_CONSTRAINT_HPP_
#define GICP_LOCALIZER__TTL_TRACK_CONSTRAINT_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace gicp_localizer {

// Scale the per-scan TTL-normal soft cap with distance travelled since the
// last accepted scan. A missing/invalid wheel sample keeps the configured
// fallback instead of relaxing the gate from an unreliable speed estimate.
inline double
speedAdaptiveLateralLimit(bool enabled, bool speed_fresh, double speed_mps,
                          double scan_dt_s, double fallback_limit_m,
                          double base_limit_m, double speed_distance_ratio,
                          double max_limit_m, double max_scan_dt_s) {
  if (!enabled || !speed_fresh || !std::isfinite(speed_mps) ||
      speed_mps < 0.0 || !std::isfinite(scan_dt_s) || scan_dt_s <= 0.0 ||
      !std::isfinite(base_limit_m) || base_limit_m <= 0.0 ||
      !std::isfinite(speed_distance_ratio) || speed_distance_ratio < 0.0 ||
      !std::isfinite(max_limit_m) || max_limit_m < base_limit_m ||
      !std::isfinite(max_scan_dt_s) || max_scan_dt_s <= 0.0) {
    return fallback_limit_m;
  }
  const double bounded_dt = std::min(scan_dt_s, max_scan_dt_s);
  return std::clamp(base_limit_m +
                        speed_distance_ratio * speed_mps * bounded_dt,
                    base_limit_m, max_limit_m);
}

// Convert the scan's marginal tangent information into the fraction of the
// GICP along-track correction that may be applied. Dividing by inlier count
// makes thresholds portable across scan point budgets. Weak/invalid geometry
// falls back to the wheel/IMU prediction (min_gain); strongly observable
// geometry retains the full LiDAR correction.
inline double
informationAdaptiveAlongGain(bool enabled, double tangent_marginal_stiffness,
                             int correspondences,
                             double low_information_per_correspondence,
                             double full_information_per_correspondence,
                             double min_gain, double configured_gain) {
  if (!enabled)
    return configured_gain;
  if (!std::isfinite(min_gain) || !std::isfinite(configured_gain) ||
      !std::isfinite(low_information_per_correspondence) ||
      !std::isfinite(full_information_per_correspondence) ||
      full_information_per_correspondence <=
          low_information_per_correspondence) {
    return configured_gain;
  }
  const double bounded_min_gain = std::clamp(min_gain, 0.0, configured_gain);
  if (!std::isfinite(tangent_marginal_stiffness) ||
      tangent_marginal_stiffness < 0.0 || correspondences <= 0) {
    return bounded_min_gain;
  }
  const double normalized =
      tangent_marginal_stiffness / static_cast<double>(correspondences);
  const double alpha =
      std::clamp((normalized - low_information_per_correspondence) /
                     (full_information_per_correspondence -
                      low_information_per_correspondence),
                 0.0, 1.0);
  return bounded_min_gain + alpha * (configured_gain - bounded_min_gain);
}

// Select the active lap-time calibration segment. A breakpoint starts a new
// segment, so values before the first breakpoint use the first entry and
// stale/invalid telemetry falls back to the base parameter.
inline int trackTimeTuningIndex(double time_on_track_s,
                                const std::vector<double> &breakpoints_s) {
  if (!std::isfinite(time_on_track_s) || time_on_track_s < 0.0 ||
      breakpoints_s.empty()) {
    return -1;
  }
  const auto it = std::upper_bound(breakpoints_s.begin(), breakpoints_s.end(),
                                   time_on_track_s);
  if (it == breakpoints_s.begin()) {
    return 0;
  }
  return static_cast<int>(std::distance(breakpoints_s.begin(), it) - 1);
}

inline double trackTimeTunedValue(double base_value, double time_on_track_s,
                                  const std::vector<double> &breakpoints_s,
                                  const std::vector<double> &values) {
  const int index = trackTimeTuningIndex(time_on_track_s, breakpoints_s);
  if (index < 0 || values.size() != breakpoints_s.size()) {
    return base_value;
  }
  return values[static_cast<size_t>(index)];
}

// Lightweight reader/evaluator for race_common Target Trajectory Line CSVs.
// It intentionally depends only on Eigen/STL so the standalone localizer can
// consume the vehicle's authoritative TTL assets without linking the complete
// race_common stack.
struct TtlTrackConstraintConfig {
  // Optional basename of the active race_common TTL CSV. Empty loads every
  // line for backwards compatibility; production overlays should select the
  // line commanded by race_common instead of treating archived alternatives
  // as simultaneously valid corridors.
  std::string line_name;
  // Dynamic race_common mode starts fail-closed until a valid active TTL
  // index is selected. The geometry module remains ROS-independent; the node
  // adapter supplies TargetTrajectoryCommand.current_ttl_index.
  bool require_active_line = false;
  double corridor_margin_m = 0.75;
  double prior_max_distance_m = 8.0;
  double max_along_correction_m = 3.0;
  double along_correction_gain = 1.0;
  double max_lateral_correction_m = 3.0;
  // Optional absolute cross-track gate measured from an official TTL
  // centerline. Values <= 0 disable it. Unlike max_lateral_correction_m,
  // this catches a prior and candidate that have already drifted together.
  double max_centerline_distance_m = 0.0;
  // Applied-pose safety cap in the TTL normal direction. Values <= 0 keep
  // the historical reject-only behavior. Small supported corrections pass
  // unchanged; a larger in-corridor wrong-basin jump is clipped and rescored.
  double max_applied_lateral_correction_m = 0.0;
  double max_heading_error_rad = 1.0471975511965976; // 60 deg
  double elevation_max_distance_m = 8.0;
  double elevation_min_span_m = 1.0;
};

struct TtlTrackProjection {
  bool valid = false;
  bool elevation_valid = false;
  std::string line_name;
  size_t segment_index = 0;
  double segment_fraction = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double heading_rad = 0.0;
  double heading_error_rad = std::numeric_limits<double>::infinity();
  double lateral_m = 0.0;
  double left_width_m = 0.0;
  double right_width_m = 0.0;
  double center_distance_m = std::numeric_limits<double>::infinity();
  double s_m = 0.0;
  double total_length_m = 0.0;
  bool closed = false;
};

enum class TtlTrackRejectReason {
  kNone = 0,
  kNoLines,
  kPriorOutsideSearch,
  kCandidateOutsideCorridor,
  kCandidateOutsideCenterlineGate,
  kHeadingMismatch,
  kAlongCorrection,
  kLateralCorrection,
};

const char *ttlTrackRejectReasonName(TtlTrackRejectReason reason);

struct TtlTrackDecision {
  bool accepted = false;
  TtlTrackRejectReason reason = TtlTrackRejectReason::kNoLines;
  TtlTrackProjection prior;
  TtlTrackProjection candidate;
  double along_correction_m = std::numeric_limits<double>::infinity();
  double raw_along_correction_m = std::numeric_limits<double>::infinity();
  double lateral_correction_m = std::numeric_limits<double>::infinity();
  double raw_lateral_correction_m = std::numeric_limits<double>::infinity();
  double corridor_excess_m = std::numeric_limits<double>::infinity();
  bool along_scaled = false;
  bool lateral_clamped = false;
  double applied_along_limit_m = 0.0;
  double applied_lateral_limit_m = 0.0;
};

class TtlTrackConstraint {
public:
  explicit TtlTrackConstraint(TtlTrackConstraintConfig config = {});

  // Loads every *.csv in directory using race_common's documented TTL CSV
  // layout: two metadata rows followed by waypoint rows. Existing data is
  // replaced only when the full directory load succeeds.
  bool loadDirectory(const std::string &directory,
                     std::string *error = nullptr);

  size_t lineCount() const { return lines_.size(); }
  size_t elevationLineCount() const;
  bool setActiveLineIndex(int index);
  int activeLineIndex() const {
    return active_line_index_.load(std::memory_order_acquire);
  }
  int transitionLineIndex() const {
    return transition_line_index_.load(std::memory_order_acquire);
  }
  bool initialLineAcquisitionPending() const {
    return initial_line_acquisition_pending_.load(std::memory_order_acquire);
  }

  // Candidate and prior are map<-base poses at the same scan timestamp.
  // The TTL never supplies an XY correction: it only accepts/rejects the
  // LiDAR candidate if one official trajectory corridor supports both poses
  // with a physically small correction along and across the track.
  TtlTrackDecision evaluate(const Eigen::Matrix4f &prior,
                            const Eigen::Matrix4f &candidate,
                            double allowed_along_correction_m =
                                std::numeric_limits<double>::quiet_NaN()) const;

  // Split the LiDAR translation update in the selected TTL's tangent/normal
  // basis. The tangent component is scaled relative to the IMU/wheel prior;
  // the normal component remains LiDAR-derived but may be hard-capped to stop
  // a wrong parallel structure from moving the vehicle across the track. No
  // TTL center-line position and no live GNSS position is injected.
  TtlTrackDecision constrainTranslation(
      const Eigen::Matrix4f &prior, Eigen::Matrix4f *candidate,
      double applied_lateral_limit_m = std::numeric_limits<double>::quiet_NaN(),
      double along_correction_gain =
          std::numeric_limits<double>::quiet_NaN(),
      double allowed_along_correction_m =
          std::numeric_limits<double>::quiet_NaN(),
      double applied_along_limit_m =
          std::numeric_limits<double>::quiet_NaN()) const;

  // Keep a dead-reckoned fallback pose inside the configured absolute TTL
  // envelope. Only XY normal to the nearest segment is changed; along-track
  // position, orientation and Z are untouched. Returns true when clamped.
  bool constrainLateralEnvelope(
      Eigen::Matrix4f *pose,
      TtlTrackProjection *original_projection = nullptr) const;

  // Nearest elevation-capable TTL projection across all loaded lines. This is
  // independent of the active XY line because some command/pit TTLs are flat
  // in Z while nearby lines carry the shared static elevation profile. Callers
  // calibrate one map_z - ttl_z datum offset, then consume no live GNSS Z.
  TtlTrackProjection projectElevation(const Eigen::Matrix4f &pose) const;

private:
  struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double left_x = 0.0;
    double left_y = 0.0;
    double right_x = 0.0;
    double right_y = 0.0;
  };

  struct Segment {
    Waypoint first;
    Waypoint second;
    double dx = 0.0;
    double dy = 0.0;
    double length = 0.0;
    double heading_rad = 0.0;
    double s0_m = 0.0;
  };

  struct Line {
    int index = -1;
    std::string name;
    std::vector<Segment> segments;
    double total_length_m = 0.0;
    bool closed = false;
    bool elevation_valid = false;
  };

  TtlTrackProjection projectOnLine(const Line &line, double x, double y,
                                   double yaw_rad) const;
  static double poseYaw(const Eigen::Matrix4f &pose);
  static double wrappedDistance(double from_s, double to_s, double length,
                                bool closed);

  TtlTrackConstraintConfig config_;
  std::vector<Line> lines_;
  std::atomic<int> active_line_index_{-1};
  // RDE changes current_ttl_index when it commands a lane change, before the
  // vehicle has physically reached the new centerline. Retain the previous
  // line only while crossing between the two official corridors. Once the
  // candidate reaches the commanded centerline gate, strict single-line
  // validation resumes automatically.
  mutable std::atomic<int> transition_line_index_{-1};
  // A replay or live node can start after race_common has already selected a
  // trajectory.  The first command therefore does not prove that the vehicle
  // is already within the configured centerline gate.  Keep the official
  // corridor, heading and per-frame correction gates active while suspending
  // only the absolute centerline check until geometry confirms acquisition.
  mutable std::atomic<bool> initial_line_acquisition_pending_{false};
};

} // namespace gicp_localizer

#endif // GICP_LOCALIZER__TTL_TRACK_CONSTRAINT_HPP_
