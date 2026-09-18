#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gicp_localizer {

// Deterministic sensor-stamp rate limiter. It advances on the input timestamp
// grid instead of callback wall time, so replay speed and executor jitter do
// not change which scans are selected. This class is intentionally not
// thread-safe; the front synchronizer protects it with sync_mtx_.
class RegistrationRateLimiter {
public:
  void configure(double rate_hz) {
    period_ns_ = (std::isfinite(rate_hz) && rate_hz > 0.0)
        ? std::max<int64_t>(1, std::llround(1.0e9 / rate_hz))
        : 0;
    reset();
  }

  void reset() {
    initialized_ = false;
    last_stamp_ns_ = 0;
    next_stamp_ns_ = 0;
  }

  bool admit(int64_t stamp_ns) {
    if (period_ns_ == 0) return true;

    // A rosbag loop/epoch change must not starve the new epoch behind the old
    // schedule. Accept its first scan and establish a fresh grid.
    if (!initialized_ || stamp_ns < last_stamp_ns_) {
      initialized_ = true;
      last_stamp_ns_ = stamp_ns;
      next_stamp_ns_ = stamp_ns + period_ns_;
      return true;
    }

    last_stamp_ns_ = stamp_ns;
    if (stamp_ns < next_stamp_ns_) return false;

    // Preserve the original phase even when an input scan is absent. This
    // avoids accumulating timestamp jitter by scheduling from the last scan.
    const int64_t periods = (stamp_ns - next_stamp_ns_) / period_ns_ + 1;
    next_stamp_ns_ += periods * period_ns_;
    return true;
  }

  bool enabled() const { return period_ns_ > 0; }

private:
  int64_t period_ns_ = 0;
  int64_t last_stamp_ns_ = 0;
  int64_t next_stamp_ns_ = 0;
  bool initialized_ = false;
};

}  // namespace gicp_localizer
