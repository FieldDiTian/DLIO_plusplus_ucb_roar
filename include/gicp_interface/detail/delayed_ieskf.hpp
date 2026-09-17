#pragma once

#include "gicp_interface/detail/planar_ieskf.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

namespace gicp_localizer::detail {

struct DelayedUpdateResult {
  IteratedUpdateResult update;
  FilterState measurement_time_state;
  FilterState latest_state;
  std::size_t replayed_imu_samples{0};
  double delay_s{0.0};
};

struct DelayedSpatialUpdateResult {
  SpatialIteratedUpdateResult update;
  FilterState measurement_time_state;
  FilterState latest_state;
  std::size_t replayed_imu_samples{0};
  double delay_s{0.0};
};

class DelayedIeskf {
public:
  DelayedIeskf(const FilterState &initial_state,
               const ImuSample &initial_sample, double maximum_history_s = 0.5,
               const ImuNoise &noise = ImuNoise{},
               const Eigen::Vector3d &gravity_world =
                   Eigen::Vector3d(0.0, 0.0, -9.80665));

  void reset(const FilterState &state, const ImuSample &sample);
  void push_imu(const ImuSample &sample);
  ForwardSpeedUpdateResult apply_latest_forward_speed(double speed_mps,
                                                      double variance_mps2);
  PlanarVelocityUpdateResult
  apply_latest_body_planar_velocity(const Eigen::Vector2d &velocity_body_mps,
                                    const Eigen::Vector2d &variance_mps2);
  void apply_latest_vertical_position(double position_z_m,
                                      double position_variance_m2,
                                      double velocity_variance_mps2);
  VerticalPositionUpdateResult
  apply_latest_vertical_observation(double position_z_m,
                                    double position_variance_m2);

  DelayedUpdateResult apply_delayed_planar_update(
      double measurement_time_s, const MeasurementBuilder &measurement_builder,
      const IteratedUpdateConfig &config = IteratedUpdateConfig{});

  DelayedSpatialUpdateResult apply_delayed_spatial_update(
      double measurement_time_s,
      const SpatialMeasurementBuilder &measurement_builder,
      const IteratedUpdateConfig &config = IteratedUpdateConfig{});

  const FilterState &state() const noexcept;
  FilterState state_at(double stamp_s) const;
  ImuTrajectory trajectory(double start_time_s, double end_time_s) const;
  std::uint64_t epoch() const noexcept;
  std::size_t history_size() const noexcept;
  double oldest_time_s() const;
  double newest_time_s() const;

private:
  struct VerticalReference {
    double position_m{0.0};
    double position_variance_m2{0.0};
    double velocity_variance_mps2{0.0};
  };

  struct PlanarVelocityReference {
    Eigen::Vector2d velocity_body_mps{Eigen::Vector2d::Zero()};
    Eigen::Vector2d variance_mps2{Eigen::Vector2d::Zero()};
  };

  struct VerticalPositionObservation {
    double position_m{0.0};
    double variance_m2{0.0};
  };

  struct HistoryEntry {
    ImuSample sample;
    FilterState state;
    std::optional<std::pair<double, double>> forward_speed;
    std::optional<PlanarVelocityReference> planar_velocity;
    std::optional<VerticalReference> vertical_reference;
    std::optional<VerticalPositionObservation> vertical_observation;
  };

  struct DelayedReplayContext {
    PlanarIeskf corrected;
    ImuSample measurement_sample;
    std::size_t replay_start_index{0};
    double delay_s{0.0};
  };

  void prune_history();
  DelayedReplayContext prepare_delayed_update(double measurement_time_s);
  void replay_and_commit(DelayedReplayContext &context,
                         FilterState &measurement_time_state,
                         FilterState &latest_state,
                         std::size_t &replayed_imu_samples);

  PlanarIeskf filter_;
  std::deque<HistoryEntry> history_;
  double maximum_history_s_{0.5};
  ImuNoise noise_;
  Eigen::Vector3d gravity_world_;
};

} // namespace gicp_localizer::detail
