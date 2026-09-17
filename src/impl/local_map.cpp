#include "gicp_interface/detail/localizer_utils.hpp"

#include <algorithm>
#include <exception>

void gicp_localizer::GicpLocalizer::initializeLocalMapTarget() {
  this->global_gicp_target_ = this->gicp.getPreparedTarget();
  if (!this->global_gicp_target_) {
    throw std::runtime_error("complete GICP map target is not prepared");
  }
  this->local_map_current_target_points_ =
      this->global_gicp_target_->cloud->size();
  if (!this->local_map_enabled_) {
    return;
  }

  const double scan_support_radius =
      this->scan_max_range_ > 0.0 ? this->scan_max_range_ : this->crop_size_;
  const double required_radius =
      scan_support_radius + this->gicp_max_corr_dist_ + 2.0;
  if (this->local_map_radius_m_ <= required_radius) {
    throw std::invalid_argument(
        "local-map radius must exceed scan support + correspondence distance "
        "+ 2m guard");
  }
  const double safe_center_travel =
      this->local_map_radius_m_ - required_radius;
  if (this->local_map_rebuild_distance_m_ >= safe_center_travel) {
    throw std::invalid_argument(
        "local-map rebuild distance must be smaller than the safe crop "
        "center travel");
  }

  const auto start = std::chrono::steady_clock::now();
  this->local_map_grid_ = std::make_unique<LocalMapGrid<PointType>>(
      this->local_map_grid_cell_size_m_);
  this->local_map_grid_->build(*this->global_gicp_target_->cloud);
  const double build_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - start)
                              .count();
  RCLCPP_INFO(
      this->get_logger(),
      "Local-map target enabled: full=%zu points, radius=%.1fm, "
      "rebuild=%.1fm, safe_center_travel=%.1fm, grid=%zu cells/%zu points "
      "(%.1fms). Startup and recovery retain the complete target.",
      this->global_gicp_target_->cloud->size(), this->local_map_radius_m_,
      this->local_map_rebuild_distance_m_, safe_center_travel,
      this->local_map_grid_->cellCount(),
      this->local_map_grid_->indexedPointCount(), build_ms);
}

void gicp_localizer::GicpLocalizer::updateLocalMapTarget(
    const Eigen::Matrix4f &guess_pose_map,
    const Eigen::Vector3f &velocity_world) {
  if (!this->local_map_enabled_ || !this->local_map_grid_ ||
      !this->global_gicp_target_ || !guess_pose_map.allFinite()) {
    return;
  }

  const Eigen::Vector2f current_center =
      guess_pose_map.block<2, 1>(0, 3);
  const double scan_support_radius =
      this->scan_max_range_ > 0.0 ? this->scan_max_range_ : this->crop_size_;
  const double safe_center_travel =
      this->local_map_radius_m_ - scan_support_radius -
      this->gicp_max_corr_dist_ - 2.0;

  // Adopt a completed target only if it still covers the complete sensor
  // support at the current seed. A slow/stale build is discarded and the
  // complete map remains active; a stale KD-tree is never queried.
  PreparedGicpTarget completed_target;
  Eigen::Vector2f completed_center = Eigen::Vector2f::Zero();
  double completed_build_ms = 0.0;
  bool completed_ready = false;
  {
    std::lock_guard<std::mutex> lock(this->local_map_pending_mtx_);
    if (this->pending_local_map_ready_) {
      completed_target = std::move(this->pending_local_gicp_target_);
      completed_center = this->pending_local_map_center_;
      completed_build_ms = this->pending_local_map_build_ms_;
      this->pending_local_map_ready_ = false;
      completed_ready = true;
    }
  }
  if (completed_ready && !this->local_map_builder_busy_.load() &&
      this->local_map_builder_thread_.joinable()) {
    this->local_map_builder_thread_.join();
  }
  if (completed_ready) {
    this->local_map_last_build_ms_ = completed_build_ms;
    if (completed_target &&
        static_cast<double>((current_center - completed_center).norm()) <=
            safe_center_travel) {
      this->active_local_gicp_target_ = std::move(completed_target);
      this->active_local_map_center_ = completed_center;
      this->active_local_map_center_valid_ = true;
      ++this->local_map_build_count_;
      RCLCPP_INFO(
          this->get_logger(),
          "Local-map target ready: center=[%.1f,%.1f], points=%zu, "
          "background_build=%.1fms",
          completed_center.x(), completed_center.y(),
          this->active_local_gicp_target_->cloud->size(), completed_build_ms);
    } else {
      ++this->local_map_stale_build_count_;
      RCLCPP_WARN(
          this->get_logger(),
          "Discarded local-map target: %s (build=%.1fms, center lag=%.1fm, "
          "safe=%.1fm); complete target remains active",
          completed_target ? "stale before adoption" : "build failed",
          completed_build_ms,
          static_cast<double>((current_center - completed_center).norm()),
          safe_center_travel);
    }
  }

  const bool recovery_requires_global = this->consecutive_failures_ >= 2;
  const bool local_target_safe =
      this->active_local_gicp_target_ &&
      this->active_local_map_center_valid_ &&
      static_cast<double>(
          (current_center - this->active_local_map_center_).norm()) <=
          safe_center_travel;
  if (!recovery_requires_global && local_target_safe) {
    if (!this->local_map_target_active_) {
      if (!this->gicp.setPreparedTarget(this->active_local_gicp_target_)) {
        throw std::runtime_error("failed to install prepared local-map target");
      }
      this->local_map_target_active_ = true;
      this->local_map_current_target_points_ =
          this->active_local_gicp_target_->cloud->size();
      ++this->local_map_switch_count_;
    }
  } else if (this->local_map_target_active_) {
    if (!this->gicp.setPreparedTarget(this->global_gicp_target_)) {
      throw std::runtime_error("failed to restore complete GICP target");
    }
    this->local_map_target_active_ = false;
    this->local_map_current_target_points_ =
        this->global_gicp_target_->cloud->size();
    ++this->local_map_global_fallback_count_;
  }

  Eigen::Vector2f desired_center = current_center;
  Eigen::Vector2f lead =
      velocity_world.head<2>() * static_cast<float>(this->local_map_lead_time_s_);
  if (!lead.allFinite()) {
    lead.setZero();
  }
  const double lead_norm = static_cast<double>(lead.norm());
  if (this->local_map_max_lead_m_ > 0.0 &&
      lead_norm > this->local_map_max_lead_m_) {
    lead *= static_cast<float>(this->local_map_max_lead_m_ / lead_norm);
  }
  desired_center += lead;

  const bool moved_since_request =
      !this->requested_local_map_center_valid_ ||
      static_cast<double>(
          (desired_center - this->requested_local_map_center_).norm()) >=
          this->local_map_rebuild_distance_m_;
  if (!moved_since_request || this->local_map_builder_busy_.load() ||
      this->local_map_builder_stop_.load()) {
    return;
  }
  if (this->local_map_builder_thread_.joinable()) {
    this->local_map_builder_thread_.join();
  }

  this->requested_local_map_center_ = desired_center;
  this->requested_local_map_center_valid_ = true;
  this->local_map_builder_busy_.store(true);
  this->local_map_builder_thread_ = std::thread([this, desired_center]() {
    const auto start = std::chrono::steady_clock::now();
    PreparedGicpTarget prepared;
    try {
      const auto indices = this->local_map_grid_->queryCircle(
          *this->global_gicp_target_->cloud, desired_center.x(),
          desired_center.y(), this->local_map_radius_m_);
      if (!this->local_map_builder_stop_.load() &&
          indices.size() >= this->local_map_min_points_) {
        prepared = GicpBackend::prepareTargetSubset(
            this->global_gicp_target_, indices,
            this->local_map_builder_threads_);
      }
    } catch (const std::exception &) {
      prepared.reset();
    }
    const double build_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();
    if (!this->local_map_builder_stop_.load()) {
      std::lock_guard<std::mutex> lock(this->local_map_pending_mtx_);
      this->pending_local_gicp_target_ = std::move(prepared);
      this->pending_local_map_center_ = desired_center;
      this->pending_local_map_build_ms_ = build_ms;
      this->pending_local_map_ready_ = true;
    }
    this->local_map_builder_busy_.store(false);
  });
}

void gicp_localizer::GicpLocalizer::stopLocalMapBuilder() {
  this->local_map_builder_stop_.store(true);
  if (this->local_map_builder_thread_.joinable() &&
      this->local_map_builder_thread_.get_id() != std::this_thread::get_id()) {
    this->local_map_builder_thread_.join();
  }
  if (this->local_map_enabled_) {
    RCLCPP_INFO(
        this->get_logger(),
        "Local-map summary: builds=%lu switches=%lu full_fallbacks=%lu "
        "stale_or_failed=%lu last_build=%.1fms target=%s/%zu points",
        this->local_map_build_count_, this->local_map_switch_count_,
        this->local_map_global_fallback_count_,
        this->local_map_stale_build_count_, this->local_map_last_build_ms_,
        this->local_map_target_active_ ? "local" : "full",
        this->local_map_current_target_points_);
  }
}
