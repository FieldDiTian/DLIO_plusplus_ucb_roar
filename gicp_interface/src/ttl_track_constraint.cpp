#include "gicp_interface/ttl_track_constraint.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace gicp_localizer {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double angle) {
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

bool parseCsvRow(const std::string &line, std::vector<double> &values) {
  values.clear();
  std::stringstream stream(line);
  std::string field;
  try {
    while (std::getline(stream, field, ',')) {
      size_t parsed = 0;
      const double value = std::stod(field, &parsed);
      if (parsed != field.size() || !std::isfinite(value))
        return false;
      values.push_back(value);
    }
  } catch (const std::exception &) {
    return false;
  }
  return !values.empty();
}

double interpolate(double first, double second, double fraction) {
  return first + fraction * (second - first);
}

} // namespace

const char *ttlTrackRejectReasonName(TtlTrackRejectReason reason) {
  switch (reason) {
  case TtlTrackRejectReason::kNone:
    return "none";
  case TtlTrackRejectReason::kNoLines:
    return "no_lines";
  case TtlTrackRejectReason::kPriorOutsideSearch:
    return "prior_outside_search";
  case TtlTrackRejectReason::kCandidateOutsideCorridor:
    return "outside_corridor";
  case TtlTrackRejectReason::kCandidateOutsideCenterlineGate:
    return "outside_centerline_gate";
  case TtlTrackRejectReason::kHeadingMismatch:
    return "heading_mismatch";
  case TtlTrackRejectReason::kAlongCorrection:
    return "along_correction";
  case TtlTrackRejectReason::kLateralCorrection:
    return "lateral_correction";
  }
  return "unknown";
}

TtlTrackConstraint::TtlTrackConstraint(TtlTrackConstraintConfig config)
    : config_(std::move(config)) {}

bool TtlTrackConstraint::loadDirectory(const std::string &directory,
                                       std::string *error) {
  namespace fs = std::filesystem;
  std::vector<Line> loaded;
  std::error_code fs_error;
  if (!fs::is_directory(directory, fs_error)) {
    if (error)
      *error = "TTL directory does not exist: " + directory;
    return false;
  }

  std::vector<fs::path> paths;
  for (const auto &entry : fs::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".csv" &&
        (config_.line_name.empty() ||
         entry.path().filename().string() == config_.line_name)) {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());

  if (!config_.line_name.empty() && paths.empty()) {
    if (error) {
      *error = "Selected TTL line does not exist: " +
               (fs::path(directory) / config_.line_name).string();
    }
    return false;
  }

  for (const auto &path : paths) {
    std::ifstream input(path);
    if (!input) {
      if (error)
        *error = "Cannot open TTL: " + path.string();
      return false;
    }

    std::string row;
    std::vector<double> values;
    if (!std::getline(input, row) || !parseCsvRow(row, values) ||
        values.size() < 2) {
      if (error)
        *error = "Malformed TTL header: " + path.string();
      return false;
    }
    const int line_index = static_cast<int>(std::llround(values[0]));
    const size_t expected_waypoints =
        static_cast<size_t>(std::llround(values[1]));
    if (!std::getline(input, row)) {
      if (error)
        *error = "Missing TTL TSP metadata row: " + path.string();
      return false;
    }

    std::vector<Waypoint> waypoints;
    waypoints.reserve(expected_waypoints);
    while (std::getline(input, row)) {
      if (row.empty())
        continue;
      if (!parseCsvRow(row, values) || values.size() < 15) {
        if (error)
          *error = "Malformed TTL waypoint in " + path.string();
        return false;
      }
      waypoints.push_back(Waypoint{values[0], values[1], values[2], values[11],
                                   values[12], values[13], values[14]});
    }
    if (waypoints.size() != expected_waypoints || waypoints.size() < 2) {
      if (error) {
        *error = "TTL waypoint count mismatch in " + path.string() +
                 ": expected " + std::to_string(expected_waypoints) +
                 ", read " + std::to_string(waypoints.size());
      }
      return false;
    }

    Line line;
    line.index = line_index;
    line.name = path.filename().string();
    const double closing_distance =
        std::hypot(waypoints.front().x - waypoints.back().x,
                   waypoints.front().y - waypoints.back().y);
    line.closed = closing_distance < 5.0;

    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto &waypoint : waypoints) {
      min_z = std::min(min_z, waypoint.z);
      max_z = std::max(max_z, waypoint.z);
    }
    line.elevation_valid = std::isfinite(min_z) && std::isfinite(max_z) &&
                           (max_z - min_z) >= config_.elevation_min_span_m;

    const size_t segment_count = waypoints.size() - 1 + (line.closed ? 1 : 0);
    line.segments.reserve(segment_count);
    double accumulated_s = 0.0;
    for (size_t i = 0; i < segment_count; ++i) {
      const Waypoint &first = waypoints[i];
      const Waypoint &second = waypoints[(i + 1) % waypoints.size()];
      const double dx = second.x - first.x;
      const double dy = second.y - first.y;
      const double length = std::hypot(dx, dy);
      if (length < 1e-6)
        continue;
      line.segments.push_back(Segment{first, second, dx, dy, length,
                                      std::atan2(dy, dx), accumulated_s});
      accumulated_s += length;
    }
    line.total_length_m = accumulated_s;
    if (line.segments.empty()) {
      if (error)
        *error = "TTL has no nonzero-length segments: " + path.string();
      return false;
    }
    loaded.push_back(std::move(line));
  }

  if (loaded.empty()) {
    if (error)
      *error = "TTL directory contains no CSV files: " + directory;
    return false;
  }
  lines_ = std::move(loaded);
  return true;
}

size_t TtlTrackConstraint::elevationLineCount() const {
  return static_cast<size_t>(
      std::count_if(lines_.begin(), lines_.end(),
                    [](const Line &line) { return line.elevation_valid; }));
}

bool TtlTrackConstraint::setActiveLineIndex(int index) {
  const bool found =
      std::any_of(lines_.begin(), lines_.end(),
                  [index](const Line &line) { return line.index == index; });
  if (!found)
    return false;
  const int previous = active_line_index_.load(std::memory_order_acquire);
  if (previous == index)
    return true;
  initial_line_acquisition_pending_.store(previous < 0,
                                          std::memory_order_release);
  transition_line_index_.store(previous, std::memory_order_release);
  active_line_index_.store(index, std::memory_order_release);
  return true;
}

TtlTrackProjection TtlTrackConstraint::projectOnLine(const Line &line, double x,
                                                     double y,
                                                     double yaw_rad) const {
  TtlTrackProjection best;
  best.line_name = line.name;
  best.total_length_m = line.total_length_m;
  best.closed = line.closed;
  best.elevation_valid = line.elevation_valid;
  double best_squared_distance = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < line.segments.size(); ++i) {
    const Segment &segment = line.segments[i];
    const double along = ((x - segment.first.x) * segment.dx +
                          (y - segment.first.y) * segment.dy) /
                         (segment.length * segment.length);
    const double fraction = std::clamp(along, 0.0, 1.0);
    const double projected_x = segment.first.x + fraction * segment.dx;
    const double projected_y = segment.first.y + fraction * segment.dy;
    const double error_x = x - projected_x;
    const double error_y = y - projected_y;
    const double squared_distance = error_x * error_x + error_y * error_y;
    if (squared_distance >= best_squared_distance)
      continue;

    const double normal_x = -segment.dy / segment.length;
    const double normal_y = segment.dx / segment.length;
    const double left_x =
        interpolate(segment.first.left_x, segment.second.left_x, fraction);
    const double left_y =
        interpolate(segment.first.left_y, segment.second.left_y, fraction);
    const double right_x =
        interpolate(segment.first.right_x, segment.second.right_x, fraction);
    const double right_y =
        interpolate(segment.first.right_y, segment.second.right_y, fraction);
    double left_width = std::abs((left_x - projected_x) * normal_x +
                                 (left_y - projected_y) * normal_y);
    double right_width = std::abs((right_x - projected_x) * normal_x +
                                  (right_y - projected_y) * normal_y);
    // A malformed/legacy boundary occasionally has little normal projection.
    // The Euclidean fallback remains conservative and uses the official point.
    if (left_width < 0.1)
      left_width = std::hypot(left_x - projected_x, left_y - projected_y);
    if (right_width < 0.1)
      right_width = std::hypot(right_x - projected_x, right_y - projected_y);

    best_squared_distance = squared_distance;
    best.valid = true;
    best.segment_index = i;
    best.segment_fraction = fraction;
    best.x = projected_x;
    best.y = projected_y;
    best.z = interpolate(segment.first.z, segment.second.z, fraction);
    best.heading_rad = segment.heading_rad;
    best.heading_error_rad = std::abs(wrapAngle(yaw_rad - segment.heading_rad));
    best.lateral_m = error_x * normal_x + error_y * normal_y;
    best.left_width_m = left_width;
    best.right_width_m = right_width;
    best.center_distance_m = std::sqrt(squared_distance);
    best.s_m = segment.s0_m + fraction * segment.length;
  }
  return best;
}

double TtlTrackConstraint::poseYaw(const Eigen::Matrix4f &pose) {
  return std::atan2(static_cast<double>(pose(1, 0)),
                    static_cast<double>(pose(0, 0)));
}

double TtlTrackConstraint::wrappedDistance(double from_s, double to_s,
                                           double length, bool closed) {
  double distance = to_s - from_s;
  if (closed && length > 1e-6) {
    while (distance > 0.5 * length)
      distance -= length;
    while (distance < -0.5 * length)
      distance += length;
  }
  return distance;
}

TtlTrackDecision
TtlTrackConstraint::evaluate(const Eigen::Matrix4f &prior,
                             const Eigen::Matrix4f &candidate,
                             double allowed_along_correction_m) const {
  TtlTrackDecision best_failure;
  if (lines_.empty())
    return best_failure;
  const double along_limit =
      std::isfinite(allowed_along_correction_m) &&
              allowed_along_correction_m > 0.0
          ? allowed_along_correction_m
          : config_.max_along_correction_m;
  const int active_line = active_line_index_.load(std::memory_order_acquire);
  if (config_.require_active_line && active_line < 0)
    return best_failure;

  int transition_line = transition_line_index_.load(std::memory_order_acquire);
  bool transitioning = transition_line >= 0 && active_line >= 0 &&
                       transition_line != active_line;
  bool acquiring_initial_line =
      initial_line_acquisition_pending_.load(std::memory_order_acquire);

  const double prior_yaw = poseYaw(prior);
  const double candidate_yaw = poseYaw(candidate);

  // current_ttl_index is a commanded maneuver line, not proof that the car is
  // already on that centerline. End the transition only after both the prior
  // and candidate have geometrically acquired the commanded line. Until then,
  // the official-corridor and per-frame correction checks remain in force,
  // while the absolute centerline gate is suspended between old and new.
  if ((transitioning || acquiring_initial_line) &&
      config_.max_centerline_distance_m > 0.0) {
    const auto active_it = std::find_if(
        lines_.begin(), lines_.end(),
        [active_line](const Line &line) { return line.index == active_line; });
    if (active_it != lines_.end()) {
      const auto prior_on_active =
          projectOnLine(*active_it, prior(0, 3), prior(1, 3), prior_yaw);
      const auto candidate_on_active = projectOnLine(
          *active_it, candidate(0, 3), candidate(1, 3), candidate_yaw);
      if (prior_on_active.valid && candidate_on_active.valid &&
          prior_on_active.center_distance_m <=
              config_.max_centerline_distance_m &&
          candidate_on_active.center_distance_m <=
              config_.max_centerline_distance_m) {
        if (acquiring_initial_line) {
          initial_line_acquisition_pending_.store(false,
                                                  std::memory_order_release);
          acquiring_initial_line = false;
        } else {
          int expected = transition_line;
          transition_line_index_.compare_exchange_strong(
              expected, -1, std::memory_order_acq_rel);
          transition_line = -1;
          transitioning = false;
        }
      }
    }
  }

  double best_failure_score = std::numeric_limits<double>::infinity();
  double best_accept_score = std::numeric_limits<double>::infinity();
  TtlTrackDecision best_accept;

  for (const Line &line : lines_) {
    if (active_line >= 0 && line.index != active_line &&
        (!transitioning || line.index != transition_line))
      continue;
    const auto prior_projection =
        projectOnLine(line, prior(0, 3), prior(1, 3), prior_yaw);
    const auto candidate_projection =
        projectOnLine(line, candidate(0, 3), candidate(1, 3), candidate_yaw);
    if (!prior_projection.valid || !candidate_projection.valid)
      continue;

    TtlTrackDecision decision;
    decision.prior = prior_projection;
    decision.candidate = candidate_projection;
    decision.along_correction_m =
        wrappedDistance(prior_projection.s_m, candidate_projection.s_m,
                        line.total_length_m, line.closed);
    decision.raw_along_correction_m = decision.along_correction_m;
    decision.lateral_correction_m =
        candidate_projection.lateral_m - prior_projection.lateral_m;
    decision.raw_lateral_correction_m = decision.lateral_correction_m;
    decision.corridor_excess_m = std::max(
        {0.0,
         candidate_projection.lateral_m - candidate_projection.left_width_m -
             config_.corridor_margin_m,
         -candidate_projection.lateral_m - candidate_projection.right_width_m -
             config_.corridor_margin_m});

    if (prior_projection.center_distance_m > config_.prior_max_distance_m) {
      decision.reason = TtlTrackRejectReason::kPriorOutsideSearch;
    } else if (prior_projection.heading_error_rad >
                   config_.max_heading_error_rad ||
               candidate_projection.heading_error_rad >
                   config_.max_heading_error_rad) {
      decision.reason = TtlTrackRejectReason::kHeadingMismatch;
    } else if (decision.corridor_excess_m > 0.0) {
      decision.reason = TtlTrackRejectReason::kCandidateOutsideCorridor;
    } else if (!transitioning && !acquiring_initial_line &&
               config_.max_centerline_distance_m > 0.0 &&
               candidate_projection.center_distance_m >
                   config_.max_centerline_distance_m) {
      decision.reason = TtlTrackRejectReason::kCandidateOutsideCenterlineGate;
    } else if (std::abs(decision.along_correction_m) > along_limit) {
      decision.reason = TtlTrackRejectReason::kAlongCorrection;
    } else if (std::abs(decision.lateral_correction_m) >
               config_.max_lateral_correction_m) {
      decision.reason = TtlTrackRejectReason::kLateralCorrection;
    } else {
      decision.accepted = true;
      decision.reason = TtlTrackRejectReason::kNone;
      const double score = candidate_projection.center_distance_m +
                           0.1 * std::abs(decision.along_correction_m) +
                           0.1 * std::abs(decision.lateral_correction_m);
      if (score < best_accept_score) {
        best_accept_score = score;
        best_accept = decision;
      }
      continue;
    }

    const double failure_score = prior_projection.center_distance_m +
                                 candidate_projection.center_distance_m +
                                 10.0 * decision.corridor_excess_m;
    if (failure_score < best_failure_score) {
      best_failure_score = failure_score;
      best_failure = decision;
    }
  }

  return std::isfinite(best_accept_score) ? best_accept : best_failure;
}

TtlTrackDecision TtlTrackConstraint::constrainTranslation(
    const Eigen::Matrix4f &prior, Eigen::Matrix4f *candidate,
    double applied_lateral_limit_m, double along_correction_gain,
    double allowed_along_correction_m,
    double applied_along_limit_m) const {
  if (candidate == nullptr)
    return {};
  const double allowed_along_limit =
      std::isfinite(allowed_along_correction_m) &&
              allowed_along_correction_m > 0.0
          ? allowed_along_correction_m
          : config_.max_along_correction_m;
  const double applied_along_limit =
      std::isfinite(applied_along_limit_m) && applied_along_limit_m > 0.0
          ? std::min(applied_along_limit_m, allowed_along_limit)
          : allowed_along_limit;
  TtlTrackDecision raw =
      evaluate(prior, *candidate, allowed_along_limit);
  const double lateral_limit =
      std::isfinite(applied_lateral_limit_m) && applied_lateral_limit_m > 0.0
          ? std::min(applied_lateral_limit_m, config_.max_lateral_correction_m)
          : config_.max_applied_lateral_correction_m;
  raw.applied_lateral_limit_m = lateral_limit;
  raw.applied_along_limit_m = applied_along_limit;
  const double effective_along_gain =
      std::isfinite(along_correction_gain)
          ? std::clamp(along_correction_gain, 0.0, 1.0)
          : config_.along_correction_gain;
  if (!raw.prior.valid || !raw.candidate.valid ||
      !std::isfinite(raw.along_correction_m) ||
      !std::isfinite(raw.lateral_correction_m) || !raw.accepted) {
    return raw;
  }

  bool along_scaled = false;
  bool lateral_clamped = false;
  if (effective_along_gain < 1.0) {
    const double applied_along = std::clamp(
        raw.along_correction_m * effective_along_gain,
        -applied_along_limit, applied_along_limit);
    const double removed_along = raw.along_correction_m - applied_along;
    if (std::abs(removed_along) > 1e-9) {
      (*candidate)(0, 3) -= static_cast<float>(
          removed_along * std::cos(raw.candidate.heading_rad));
      (*candidate)(1, 3) -= static_cast<float>(
          removed_along * std::sin(raw.candidate.heading_rad));
      along_scaled = true;
    }
  }

  TtlTrackDecision constrained =
      along_scaled ? evaluate(prior, *candidate, allowed_along_limit) : raw;
  constrained.applied_along_limit_m = applied_along_limit;
  constrained.applied_lateral_limit_m = lateral_limit;
  if (constrained.accepted && constrained.prior.valid &&
      constrained.candidate.valid && lateral_limit > 0.0 &&
      std::isfinite(constrained.lateral_correction_m)) {
    const double applied_lateral = std::clamp(constrained.lateral_correction_m,
                                              -lateral_limit, lateral_limit);
    const double removed_lateral =
        constrained.lateral_correction_m - applied_lateral;
    if (std::abs(removed_lateral) > 1e-9) {
      const double normal_x = -std::sin(constrained.candidate.heading_rad);
      const double normal_y = std::cos(constrained.candidate.heading_rad);
      (*candidate)(0, 3) -= static_cast<float>(removed_lateral * normal_x);
      (*candidate)(1, 3) -= static_cast<float>(removed_lateral * normal_y);
      lateral_clamped = true;
      constrained = evaluate(prior, *candidate, allowed_along_limit);
      constrained.applied_along_limit_m = applied_along_limit;
      constrained.applied_lateral_limit_m = lateral_limit;
    }
  }

  constrained.raw_along_correction_m = raw.along_correction_m;
  constrained.raw_lateral_correction_m = raw.lateral_correction_m;
  constrained.along_scaled = along_scaled;
  constrained.lateral_clamped = lateral_clamped;
  constrained.applied_along_limit_m = applied_along_limit;
  constrained.applied_lateral_limit_m = lateral_limit;
  return constrained;
}

bool TtlTrackConstraint::constrainLateralEnvelope(
    Eigen::Matrix4f *pose, TtlTrackProjection *original_projection) const {
  if (pose == nullptr || config_.max_centerline_distance_m <= 0.0 ||
      lines_.empty()) {
    return false;
  }

  const double yaw = poseYaw(*pose);
  const int active_line = active_line_index_.load(std::memory_order_acquire);
  if (config_.require_active_line && active_line < 0)
    return false;
  // Clamping to either centerline during a commanded line change would
  // prevent the physical maneuver. Evaluation still enforces the official
  // road corridor and the per-frame correction bounds during this interval.
  if (transition_line_index_.load(std::memory_order_acquire) >= 0 ||
      initial_line_acquisition_pending_.load(std::memory_order_acquire))
    return false;
  TtlTrackProjection best;
  for (const Line &line : lines_) {
    if (active_line >= 0 && line.index != active_line)
      continue;
    const auto projection =
        projectOnLine(line, (*pose)(0, 3), (*pose)(1, 3), yaw);
    if (projection.valid && (!best.valid || projection.center_distance_m <
                                                best.center_distance_m)) {
      best = projection;
    }
  }
  if (original_projection != nullptr)
    *original_projection = best;
  if (!best.valid || !std::isfinite(best.center_distance_m) ||
      best.center_distance_m <= config_.max_centerline_distance_m ||
      best.center_distance_m <= 1e-9) {
    return false;
  }

  const double scale =
      config_.max_centerline_distance_m / best.center_distance_m;
  (*pose)(0, 3) = static_cast<float>(
      best.x + (static_cast<double>((*pose)(0, 3)) - best.x) * scale);
  (*pose)(1, 3) = static_cast<float>(
      best.y + (static_cast<double>((*pose)(1, 3)) - best.y) * scale);
  return true;
}

TtlTrackProjection
TtlTrackConstraint::projectElevation(const Eigen::Matrix4f &pose) const {
  TtlTrackProjection best;
  const double yaw = poseYaw(pose);
  // Elevation is a static geometric profile, not a driveable-corridor choice.
  // Laguna's commanded TTL 27 is authoritative for XY gating but contains a
  // deliberately flat Z column. Nearby racing-line TTLs share the same ENU
  // datum and contain the real elevation profile. Search every
  // elevation-capable line here while constrainTranslation() continues to
  // enforce only the active/transition TTL for XY.
  for (const Line &line : lines_) {
    if (!line.elevation_valid)
      continue;
    auto projection = projectOnLine(line, pose(0, 3), pose(1, 3), yaw);
    if (!projection.valid ||
        projection.center_distance_m > config_.elevation_max_distance_m ||
        projection.heading_error_rad > config_.max_heading_error_rad) {
      continue;
    }
    if (!best.valid || projection.center_distance_m < best.center_distance_m) {
      best = std::move(projection);
    }
  }
  return best;
}

} // namespace gicp_localizer
