#include "gicp_interface/detail/delayed_ieskf.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gicp_localizer::detail {
namespace {

constexpr double kTimestampToleranceS = 1e-9;

ImuSample interpolate_sample(const ImuSample &before, const ImuSample &after,
                             double stamp_s) {
  if (stamp_s < before.stamp_s - kTimestampToleranceS ||
      stamp_s > after.stamp_s + kTimestampToleranceS ||
      after.stamp_s <= before.stamp_s) {
    throw std::invalid_argument("cannot interpolate outside IMU bracket");
  }
  const double alpha =
      (stamp_s - before.stamp_s) / (after.stamp_s - before.stamp_s);
  return ImuSample{
      stamp_s,
      before.angular_velocity +
          alpha * (after.angular_velocity - before.angular_velocity),
      before.linear_acceleration +
          alpha * (after.linear_acceleration - before.linear_acceleration),
      std::nullopt, std::nullopt};
}

void validate_initial_pair(const FilterState &state, const ImuSample &sample) {
  if (!std::isfinite(sample.stamp_s) || !sample.angular_velocity.allFinite() ||
      !sample.linear_acceleration.allFinite() ||
      std::abs(sample.stamp_s - state.nominal.stamp_s) > kTimestampToleranceS) {
    throw std::invalid_argument(
        "initial IMU sample must match filter timestamp");
  }
}

} // namespace

DelayedIeskf::DelayedIeskf(const FilterState &initial_state,
                           const ImuSample &initial_sample,
                           double maximum_history_s, const ImuNoise &noise,
                           const Eigen::Vector3d &gravity_world)
    : filter_(initial_state, noise, gravity_world),
      maximum_history_s_(maximum_history_s), noise_(noise),
      gravity_world_(gravity_world) {
  if (!std::isfinite(maximum_history_s_) || maximum_history_s_ <= 0.0) {
    throw std::invalid_argument(
        "maximum filter history must be positive and finite");
  }
  reset(initial_state, initial_sample);
}

void DelayedIeskf::reset(const FilterState &state, const ImuSample &sample) {
  validate_initial_pair(state, sample);
  filter_.reset(state);
  history_.clear();
  history_.push_back(
      HistoryEntry{sample, state, std::nullopt, std::nullopt, std::nullopt,
                   std::nullopt});
}

void DelayedIeskf::push_imu(const ImuSample &sample) {
  if (history_.empty()) {
    throw std::logic_error("delayed filter has no reset anchor");
  }
  const ImuSample &previous = history_.back().sample;
  filter_.propagate(previous, sample);
  if (sample.orientation_world_from_body.has_value()) {
    if (!sample.orientation_variance_rad2.has_value()) {
      throw std::invalid_argument("IMU orientation is missing its variance");
    }
    filter_.pin_orientation(*sample.orientation_world_from_body,
                            *sample.orientation_variance_rad2);
  }
  history_.push_back(HistoryEntry{sample, filter_.state(), std::nullopt,
                                  std::nullopt, std::nullopt, std::nullopt});
  prune_history();
}

ForwardSpeedUpdateResult
DelayedIeskf::apply_latest_forward_speed(double speed_mps,
                                         double variance_mps2) {
  if (history_.empty()) {
    throw std::logic_error("delayed filter has no speed-update anchor");
  }
  const ForwardSpeedUpdateResult result =
      filter_.update_forward_speed(speed_mps, variance_mps2);
  history_.back().state = filter_.state();
  history_.back().forward_speed = std::make_pair(speed_mps, variance_mps2);
  return result;
}

PlanarVelocityUpdateResult DelayedIeskf::apply_latest_body_planar_velocity(
    const Eigen::Vector2d &velocity_body_mps,
    const Eigen::Vector2d &variance_mps2) {
  if (history_.empty()) {
    throw std::logic_error("delayed filter has no velocity-update anchor");
  }
  const PlanarVelocityUpdateResult result =
      filter_.update_body_planar_velocity(velocity_body_mps, variance_mps2);
  history_.back().state = filter_.state();
  history_.back().planar_velocity =
      PlanarVelocityReference{velocity_body_mps, variance_mps2};
  return result;
}

void DelayedIeskf::apply_latest_vertical_position(
    double position_z_m, double position_variance_m2,
    double velocity_variance_mps2) {
  if (history_.empty()) {
    throw std::logic_error("delayed filter has no vertical-reference anchor");
  }
  filter_.pin_vertical_position(position_z_m, position_variance_m2,
                                velocity_variance_mps2);
  history_.back().state = filter_.state();
  history_.back().vertical_reference = VerticalReference{
      position_z_m, position_variance_m2, velocity_variance_mps2};
}

VerticalPositionUpdateResult DelayedIeskf::apply_latest_vertical_observation(
    double position_z_m, double position_variance_m2) {
  if (history_.empty()) {
    throw std::logic_error("delayed filter has no vertical-observation anchor");
  }
  const VerticalPositionUpdateResult result =
      filter_.update_vertical_position(position_z_m, position_variance_m2);
  history_.back().state = filter_.state();
  history_.back().vertical_observation =
      VerticalPositionObservation{position_z_m, position_variance_m2};
  return result;
}

DelayedUpdateResult DelayedIeskf::apply_delayed_planar_update(
    double measurement_time_s, const MeasurementBuilder &measurement_builder,
    const IteratedUpdateConfig &config) {
  DelayedReplayContext context = prepare_delayed_update(measurement_time_s);
  DelayedUpdateResult result;
  result.delay_s = context.delay_s;
  result.update = context.corrected.update_planar(measurement_builder, config);
  replay_and_commit(context, result.measurement_time_state, result.latest_state,
                    result.replayed_imu_samples);
  return result;
}

DelayedSpatialUpdateResult DelayedIeskf::apply_delayed_spatial_update(
    double measurement_time_s,
    const SpatialMeasurementBuilder &measurement_builder,
    const IteratedUpdateConfig &config) {
  DelayedReplayContext context = prepare_delayed_update(measurement_time_s);
  DelayedSpatialUpdateResult result;
  result.delay_s = context.delay_s;
  result.update = context.corrected.update_spatial(measurement_builder, config);
  replay_and_commit(context, result.measurement_time_state, result.latest_state,
                    result.replayed_imu_samples);
  return result;
}

DelayedIeskf::DelayedReplayContext
DelayedIeskf::prepare_delayed_update(double measurement_time_s) {
  if (history_.empty() || !std::isfinite(measurement_time_s) ||
      measurement_time_s < oldest_time_s() - kTimestampToleranceS ||
      measurement_time_s > newest_time_s() + kTimestampToleranceS) {
    throw std::out_of_range(
        "measurement time lies outside same-epoch IMU history");
  }

  const auto upper =
      std::lower_bound(history_.begin(), history_.end(), measurement_time_s,
                       [](const HistoryEntry &entry, double stamp) {
                         return entry.sample.stamp_s < stamp;
                       });
  if (upper == history_.end()) {
    throw std::out_of_range("measurement has no newer IMU bracket");
  }

  std::size_t upper_index =
      static_cast<std::size_t>(std::distance(history_.begin(), upper));
  FilterState measurement_anchor;
  ImuSample measurement_sample;
  std::size_t replay_start_index = upper_index;
  if (std::abs(upper->sample.stamp_s - measurement_time_s) <=
      kTimestampToleranceS) {
    measurement_anchor = upper->state;
    measurement_sample = upper->sample;
    replay_start_index = upper_index + 1;
  } else {
    if (upper == history_.begin()) {
      throw std::out_of_range("measurement has no older IMU bracket");
    }
    const HistoryEntry &before = *(upper - 1);
    measurement_sample =
        interpolate_sample(before.sample, upper->sample, measurement_time_s);
    PlanarIeskf interpolated(before.state, noise_, gravity_world_);
    interpolated.propagate(before.sample, measurement_sample);
    measurement_anchor = interpolated.state();
    replay_start_index = upper_index;
  }

  return DelayedReplayContext{
      PlanarIeskf(measurement_anchor, noise_, gravity_world_),
      measurement_sample, replay_start_index,
      newest_time_s() - measurement_time_s};
}

void DelayedIeskf::replay_and_commit(DelayedReplayContext &context,
                                     FilterState &measurement_time_state,
                                     FilterState &latest_state,
                                     std::size_t &replayed_imu_samples) {
  // The caller performs the map update on a copied filter before entering this
  // method. A rejected or degenerate update therefore cannot mutate history.
  measurement_time_state = context.corrected.state();
  ImuSample previous_sample = context.measurement_sample;
  for (std::size_t i = context.replay_start_index; i < history_.size(); ++i) {
    context.corrected.propagate(previous_sample, history_[i].sample);
    if (history_[i].sample.orientation_world_from_body.has_value()) {
      context.corrected.pin_orientation(
          *history_[i].sample.orientation_world_from_body,
          *history_[i].sample.orientation_variance_rad2);
    }
    if (history_[i].vertical_reference.has_value()) {
      const VerticalReference &reference = *history_[i].vertical_reference;
      context.corrected.pin_vertical_position(reference.position_m,
                                              reference.position_variance_m2,
                                              reference.velocity_variance_mps2);
    }
    if (history_[i].vertical_observation.has_value()) {
      const VerticalPositionObservation &observation =
          *history_[i].vertical_observation;
      context.corrected.update_vertical_position(observation.position_m,
                                                 observation.variance_m2);
    }
    if (history_[i].forward_speed.has_value()) {
      context.corrected.update_forward_speed(history_[i].forward_speed->first,
                                             history_[i].forward_speed->second);
    }
    if (history_[i].planar_velocity.has_value()) {
      const PlanarVelocityReference &reference = *history_[i].planar_velocity;
      context.corrected.update_body_planar_velocity(reference.velocity_body_mps,
                                                    reference.variance_mps2);
    }
    history_[i].state = context.corrected.state();
    previous_sample = history_[i].sample;
    ++replayed_imu_samples;
  }

  if (context.replay_start_index > 0 &&
      std::abs(history_[context.replay_start_index - 1].sample.stamp_s -
               context.measurement_sample.stamp_s) <= kTimestampToleranceS) {
    history_[context.replay_start_index - 1].state = measurement_time_state;
  }
  filter_ = context.corrected;
  latest_state = filter_.state();
}

const FilterState &DelayedIeskf::state() const noexcept {
  return filter_.state();
}

FilterState DelayedIeskf::state_at(double stamp_s) const {
  if (history_.empty() || !std::isfinite(stamp_s) ||
      stamp_s < oldest_time_s() - kTimestampToleranceS ||
      stamp_s > newest_time_s() + kTimestampToleranceS) {
    throw std::out_of_range("state time lies outside same-epoch IMU history");
  }
  const auto upper =
      std::lower_bound(history_.begin(), history_.end(), stamp_s,
                       [](const HistoryEntry &entry, double stamp) {
                         return entry.sample.stamp_s < stamp;
                       });
  if (upper == history_.end()) {
    return history_.back().state;
  }
  if (std::abs(upper->sample.stamp_s - stamp_s) <= kTimestampToleranceS) {
    return upper->state;
  }
  if (upper == history_.begin()) {
    return history_.front().state;
  }
  const HistoryEntry &before = *(upper - 1);
  const ImuSample interpolated =
      interpolate_sample(before.sample, upper->sample, stamp_s);
  PlanarIeskf filter(before.state, noise_, gravity_world_);
  filter.propagate(before.sample, interpolated);
  return filter.state();
}

ImuTrajectory DelayedIeskf::trajectory(double start_time_s,
                                       double end_time_s) const {
  if (!std::isfinite(start_time_s) || !std::isfinite(end_time_s) ||
      end_time_s < start_time_s ||
      start_time_s < oldest_time_s() - kTimestampToleranceS ||
      end_time_s > newest_time_s() + kTimestampToleranceS) {
    throw std::out_of_range(
        "trajectory range lies outside same-epoch IMU history");
  }

  const FilterState anchor = state_at(start_time_s);
  std::vector<ImuSample> samples;
  const auto start_upper =
      std::lower_bound(history_.begin(), history_.end(), start_time_s,
                       [](const HistoryEntry &entry, double stamp) {
                         return entry.sample.stamp_s < stamp;
                       });
  if (start_upper != history_.end() &&
      std::abs(start_upper->sample.stamp_s - start_time_s) <=
          kTimestampToleranceS) {
    samples.push_back(start_upper->sample);
  } else {
    if (start_upper == history_.begin() || start_upper == history_.end()) {
      throw std::out_of_range("trajectory start has no IMU bracket");
    }
    samples.push_back(interpolate_sample((start_upper - 1)->sample,
                                         start_upper->sample, start_time_s));
  }

  for (const HistoryEntry &entry : history_) {
    if (entry.sample.stamp_s > start_time_s + kTimestampToleranceS &&
        entry.sample.stamp_s < end_time_s - kTimestampToleranceS) {
      samples.push_back(entry.sample);
    }
  }
  if (end_time_s > start_time_s + kTimestampToleranceS) {
    const auto end_upper =
        std::lower_bound(history_.begin(), history_.end(), end_time_s,
                         [](const HistoryEntry &entry, double stamp) {
                           return entry.sample.stamp_s < stamp;
                         });
    if (end_upper == history_.end()) {
      samples.push_back(history_.back().sample);
    } else if (std::abs(end_upper->sample.stamp_s - end_time_s) <=
               kTimestampToleranceS) {
      samples.push_back(end_upper->sample);
    } else {
      if (end_upper == history_.begin()) {
        throw std::out_of_range("trajectory end has no IMU bracket");
      }
      samples.push_back(interpolate_sample((end_upper - 1)->sample,
                                           end_upper->sample, end_time_s));
    }
  }
  return ImuTrajectory::integrate(anchor.nominal, samples, gravity_world_);
}

std::uint64_t DelayedIeskf::epoch() const noexcept {
  return filter_.state().nominal.epoch;
}

std::size_t DelayedIeskf::history_size() const noexcept {
  return history_.size();
}

double DelayedIeskf::oldest_time_s() const {
  if (history_.empty()) {
    throw std::logic_error("empty delayed filter has no oldest time");
  }
  return history_.front().sample.stamp_s;
}

double DelayedIeskf::newest_time_s() const {
  if (history_.empty()) {
    throw std::logic_error("empty delayed filter has no newest time");
  }
  return history_.back().sample.stamp_s;
}

void DelayedIeskf::prune_history() {
  const double cutoff = newest_time_s() - maximum_history_s_;
  while (history_.size() > 2 && history_[1].sample.stamp_s < cutoff) {
    history_.pop_front();
  }
}

} // namespace gicp_localizer::detail
