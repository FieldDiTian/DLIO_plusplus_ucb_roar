#ifndef GICP_LOCALIZER_LUMINAR_SWEEP_MATCHING_HPP
#define GICP_LOCALIZER_LUMINAR_SWEEP_MATCHING_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace gicp_localizer {

struct LuminarTimestampRangeNs {
  bool valid = false;
  uint64_t min_ns = 0;
  uint64_t max_ns = 0;
  size_t count = 0;
};

inline uint64_t absDiffNs(uint64_t a, uint64_t b) {
  return (a >= b) ? (a - b) : (b - a);
}

inline uint64_t shiftedTimestampNs(uint64_t timestamp_ns, int64_t shift_ns) {
  if (shift_ns >= 0) {
    const uint64_t positive_shift = static_cast<uint64_t>(shift_ns);
    if (timestamp_ns > std::numeric_limits<uint64_t>::max() - positive_shift) {
      return std::numeric_limits<uint64_t>::max();
    }
    return timestamp_ns + positive_shift;
  }

  const uint64_t negative_shift = static_cast<uint64_t>(-(shift_ns + 1)) + 1;
  return (timestamp_ns >= negative_shift) ? timestamp_ns - negative_shift : 0;
}

inline int64_t secondsToNanoseconds(double seconds) {
  if (!std::isfinite(seconds)) {
    return 0;
  }
  const long double ns = static_cast<long double>(seconds) * 1.0e9L;
  if (ns >= static_cast<long double>(std::numeric_limits<int64_t>::max())) {
    return std::numeric_limits<int64_t>::max();
  }
  if (ns <= static_cast<long double>(std::numeric_limits<int64_t>::min())) {
    return std::numeric_limits<int64_t>::min();
  }
  return static_cast<int64_t>(std::llround(ns));
}

inline LuminarTimestampRangeNs shiftedRange(
    const LuminarTimestampRangeNs& range, double clock_offset_s) {
  if (!range.valid) {
    return range;
  }
  const int64_t shift_ns = secondsToNanoseconds(clock_offset_s);
  LuminarTimestampRangeNs shifted = range;
  shifted.min_ns = shiftedTimestampNs(range.min_ns, shift_ns);
  shifted.max_ns = shiftedTimestampNs(range.max_ns, shift_ns);
  return shifted;
}

inline double endpointDeltaSeconds(
    const LuminarTimestampRangeNs& lhs,
    const LuminarTimestampRangeNs& rhs) {
  if (!lhs.valid || !rhs.valid) {
    return std::numeric_limits<double>::infinity();
  }
  const uint64_t delta_ns = std::max(
      absDiffNs(lhs.min_ns, rhs.min_ns),
      absDiffNs(lhs.max_ns, rhs.max_ns));
  return static_cast<double>(delta_ns) * 1.0e-9;
}

struct LuminarSweepCandidate {
  size_t index = 0;
  LuminarTimestampRangeNs range;
  double header_abs_delta_s = std::numeric_limits<double>::infinity();
};

struct LuminarSweepSelection {
  size_t index = 0;
  double range_delta_s = std::numeric_limits<double>::infinity();
  double header_abs_delta_s = std::numeric_limits<double>::infinity();
};

inline std::optional<LuminarSweepSelection> selectClosestLuminarSweep(
    const LuminarTimestampRangeNs& primary,
    const std::vector<LuminarSweepCandidate>& candidates) {
  std::optional<LuminarSweepSelection> best;
  for (const auto& candidate : candidates) {
    const double range_delta_s = endpointDeltaSeconds(primary, candidate.range);
    if (!std::isfinite(range_delta_s)) {
      continue;
    }
    if (!best || range_delta_s < best->range_delta_s ||
        (range_delta_s == best->range_delta_s &&
         candidate.header_abs_delta_s < best->header_abs_delta_s)) {
      best = LuminarSweepSelection{
          candidate.index, range_delta_s, candidate.header_abs_delta_s};
    }
  }
  return best;
}

inline bool luminarWatermarkPassed(
    const LuminarTimestampRangeNs& primary,
    const LuminarTimestampRangeNs& newest_aux,
    double threshold_s) {
  if (!primary.valid || !newest_aux.valid) {
    return false;
  }
  const uint64_t threshold_ns = static_cast<uint64_t>(
      std::max<int64_t>(0, secondsToNanoseconds(threshold_s)));
  const uint64_t deadline_ns =
      primary.min_ns > std::numeric_limits<uint64_t>::max() - threshold_ns
          ? std::numeric_limits<uint64_t>::max()
          : primary.min_ns + threshold_ns;
  return newest_aux.min_ns > deadline_ns;
}

}  // namespace gicp_localizer

#endif  // GICP_LOCALIZER_LUMINAR_SWEEP_MATCHING_HPP
