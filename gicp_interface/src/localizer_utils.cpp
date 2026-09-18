/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "gicp_interface/detail/localizer_utils.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

namespace gicp_localizer::detail {

constexpr double kRadToDeg = 57.29577951308232;

bool matrixFinite(const Eigen::Matrix4f& pose) {
  return pose.array().isFinite().all();
}

geometry_msgs::msg::Pose poseMsgFromMatrix(const Eigen::Matrix4f& pose) {
  geometry_msgs::msg::Pose msg;
  const Eigen::Vector3f t = pose.block<3, 1>(0, 3);
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  q.normalize();

  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  return msg;
}

double rotationDistanceDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  Eigen::Quaternionf qa(a.block<3, 3>(0, 0));
  Eigen::Quaternionf qb(b.block<3, 3>(0, 0));
  qa.normalize();
  qb.normalize();

  Eigen::Quaternionf dq = qa.conjugate() * qb;
  dq.normalize();
  const double w = std::clamp(std::abs(static_cast<double>(dq.w())), 0.0, 1.0);
  return 2.0 * std::acos(w) * kRadToDeg;
}

double deltaTranslationNorm(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to) {
  return (to.block<3, 1>(0, 3) - from.block<3, 1>(0, 3)).norm();
}

std::string poseSummary(const Eigen::Matrix4f& pose) {
  if (!matrixFinite(pose)) {
    return "invalid";
  }

  const Eigen::Vector3f t = pose.block<3, 1>(0, 3);
  const Eigen::Vector3f rpy_deg = pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2) * static_cast<float>(kRadToDeg);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "xyz=[" << t.x() << "," << t.y() << "," << t.z() << "]"
      << " rpy_deg=[" << rpy_deg.x() << "," << rpy_deg.y() << "," << rpy_deg.z() << "]";
  return oss.str();
}

std::string scalarSummary(double value, int precision) {
  if (!std::isfinite(value)) {
    return "nan";
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

bool findXYZOffsets(const sensor_msgs::msg::PointCloud2& msg, int& x_off, int& y_off, int& z_off) {
  x_off = y_off = z_off = -1;
  for (const auto& f : msg.fields) {
    if (f.name == "x") x_off = static_cast<int>(f.offset);
    else if (f.name == "y") y_off = static_cast<int>(f.offset);
    else if (f.name == "z") z_off = static_cast<int>(f.offset);
  }
  return x_off >= 0 && y_off >= 0 && z_off >= 0;
}

// Full point-field schema comparison, used to decide whether an auxiliary cloud
// can be byte-appended onto the primary cloud. mergeAuxClouds() transforms aux
// points using the AUX cloud's own field offsets, but the MERGED cloud keeps the
// PRIMARY's `fields`, so every downstream reader interprets the appended aux
// bytes with the primary layout. Appending is therefore only safe when the aux
// layout is byte-identical to the primary: same point_step, same endianness, and
// the same ordered set of field {name, offset, datatype, count}. A same-point_step
// cloud with different offsets/datatypes would otherwise be silently misread.
// This is O(#fields) (~10-20 per Luminar scan) — negligible next to GICP / voxel
// filtering / deskew, which run in ms. Returns true on match; on mismatch returns
// false and sets `reason` to a short human-readable description for logging.
bool auxSchemaMatchesPrimary(const sensor_msgs::msg::PointCloud2& aux,
                             const sensor_msgs::msg::PointCloud2& primary,
                             std::string& reason) {
  if (aux.point_step != primary.point_step) {
    reason = "point_step " + std::to_string(aux.point_step) + " vs primary " +
             std::to_string(primary.point_step);
    return false;
  }
  if (aux.is_bigendian != primary.is_bigendian) {
    reason = "endianness differs (aux is_bigendian=" + std::to_string(aux.is_bigendian) + ")";
    return false;
  }
  if (aux.fields.size() != primary.fields.size()) {
    reason = "field count " + std::to_string(aux.fields.size()) + " vs primary " +
             std::to_string(primary.fields.size());
    return false;
  }
  for (size_t i = 0; i < primary.fields.size(); ++i) {
    const auto& a = aux.fields[i];
    const auto& p = primary.fields[i];
    if (a.name != p.name || a.offset != p.offset || a.datatype != p.datatype || a.count != p.count) {
      reason = "field[" + std::to_string(i) + "] '" + a.name + "' (offset=" + std::to_string(a.offset) +
               ",datatype=" + std::to_string(a.datatype) + ",count=" + std::to_string(a.count) +
               ") differs from primary '" + p.name + "' (offset=" + std::to_string(p.offset) +
               ",datatype=" + std::to_string(p.datatype) + ",count=" + std::to_string(p.count) + ")";
      return false;
    }
  }
  return true;
}

// Apply rigid transform to xyz of `num_points` points starting at `data` (in place).
// Pointer-based variant — operates on a span within a larger buffer so callers
// can write aux scans directly into the merged cloud's data without an
// intermediate copy.
void transformCloudData(uint8_t* data, size_t num_points, uint32_t point_step,
                        int x_off, int y_off, int z_off,
                        const Eigen::Matrix4f& T) {
  // [P3 FIX 2026-07-10] Bounds: a malformed schema can declare a field that
  // does not fit its point (offset + width > point_step) — the writes below
  // would then smear into the next point and, on the LAST point, past the
  // buffer (heap corruption). Bail instead of corrupting.
  if (x_off < 0 || y_off < 0 || z_off < 0 ||
      static_cast<uint32_t>(x_off) + sizeof(float) > point_step ||
      static_cast<uint32_t>(y_off) + sizeof(float) > point_step ||
      static_cast<uint32_t>(z_off) + sizeof(float) > point_step) {
    return;
  }
  const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  const Eigen::Vector3f t = T.block<3, 1>(0, 3);
  for (size_t i = 0; i < num_points; ++i) {
    uint8_t* base = data + i * point_step;
    float x, y, z;
    std::memcpy(&x, base + x_off, sizeof(float));
    std::memcpy(&y, base + y_off, sizeof(float));
    std::memcpy(&z, base + z_off, sizeof(float));
    Eigen::Vector3f p = R * Eigen::Vector3f(x, y, z) + t;
    std::memcpy(base + x_off, &p.x(), sizeof(float));
    std::memcpy(base + y_off, &p.y(), sizeof(float));
    std::memcpy(base + z_off, &p.z(), sizeof(float));
  }
}

const char* pointFieldDatatypeName(uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:    return "INT8";
    case sensor_msgs::msg::PointField::UINT8:   return "UINT8";
    case sensor_msgs::msg::PointField::INT16:   return "INT16";
    case sensor_msgs::msg::PointField::UINT16:  return "UINT16";
    case sensor_msgs::msg::PointField::INT32:   return "INT32";
    case sensor_msgs::msg::PointField::UINT32:  return "UINT32";
    case sensor_msgs::msg::PointField::FLOAT32: return "FLOAT32";
    case sensor_msgs::msg::PointField::FLOAT64: return "FLOAT64";
    default:                                    return "UNKNOWN";
  }
}

// One-shot diagnostic: print everything we can extract about the incoming
// PointCloud2's timestamp field so a developer can decide which bit-level
// interpretation the live driver actually uses. See
// docs/luminar_timestamp_diagnostic_guide.pdf for how to read this output.
//
// Fires only on the first cloud (guarded by std::call_once at the caller),
// always emits the lines regardless of localization/verbose so a single
// test run produces the answer.
//
// Output format (per line):
//   [LUMINAR_TS_DIAG] <key>: <value>
// The block is bracketed by [LUMINAR_TS_DIAG] BEGIN / END markers so it's
// easy to grep out of a noisy log.
void logTimestampDiagnostic(const sensor_msgs::msg::PointCloud2& pc,
                            int ts_off, uint8_t ts_datatype, int ts_count,
                            const char* sensor_name) {
  std::fprintf(stderr, "[LUMINAR_TS_DIAG] BEGIN\n");
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] sensor_type=%s  point_step=%u  "
               "num_points=%u  width=%u  height=%u  is_bigendian=%d  "
               "header.stamp=%u.%09u  frame_id='%s'\n",
               sensor_name, pc.point_step,
               static_cast<unsigned>(pc.data.size() / std::max<uint32_t>(pc.point_step, 1u)),
               pc.width, pc.height, pc.is_bigendian ? 1 : 0,
               pc.header.stamp.sec, pc.header.stamp.nanosec,
               pc.header.frame_id.c_str());
  std::fprintf(stderr, "[LUMINAR_TS_DIAG] fields (offset, datatype, count, name):\n");
  for (const auto& f : pc.fields) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   off=%-4u  type=%-7s  count=%-3u  name='%s'\n",
                 f.offset, pointFieldDatatypeName(f.datatype), f.count,
                 f.name.c_str());
  }
  if (ts_off < 0) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG] no timestamp field detected (no field named "
                 "t/time/time_stamp/timestamp). Deskew cannot use per-point times.\n");
    std::fprintf(stderr, "[LUMINAR_TS_DIAG] END\n");
    std::fflush(stderr);
    return;
  }
  // Implied byte size per datatype, times count.
  // (count is normally 1 except for UINT8 where count carries the length of
  // the byte run, e.g. UINT8 count=8 = 8 raw bytes.)
  int bytes_per_elem = 1;
  switch (ts_datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:    bytes_per_elem = 1; break;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:   bytes_per_elem = 2; break;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:  bytes_per_elem = 4; break;
    case sensor_msgs::msg::PointField::FLOAT64:  bytes_per_elem = 8; break;
    default:                                      bytes_per_elem = 0; break;
  }
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] timestamp_field: off=%d  type=%s  count=%d  "
               "implied_byte_size=%d\n",
               ts_off, pointFieldDatatypeName(ts_datatype), ts_count,
               bytes_per_elem * ts_count);

  // Walk up to 3 sample points (first, midpoint, last) and dump their 8
  // timestamp bytes interpreted four different ways.  This lets the developer
  // pattern-match what the driver actually emits without instrumenting the
  // driver itself.
  const uint32_t step = pc.point_step;
  const size_t num_points = pc.data.size() / std::max<uint32_t>(step, 1u);
  if (num_points == 0 || ts_off + 8 > static_cast<int>(step)) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG] (no samples to dump -- empty cloud or "
                 "field extends past point_step)\n");
    std::fprintf(stderr, "[LUMINAR_TS_DIAG] END\n");
    std::fflush(stderr);
    return;
  }
  const size_t sample_indices[3] = {
      0u, num_points / 2u,
      num_points > 0u ? num_points - 1u : 0u};
  const char* sample_labels[3] = {"point[0]    ", "point[mid]  ", "point[N-1]  "};

  uint64_t ts_uint64[3] = {0, 0, 0};
  double   ts_double[3] = {0.0, 0.0, 0.0};

  for (int s = 0; s < 3; ++s) {
    const size_t idx = sample_indices[s];
    const uint8_t* ptr = pc.data.data() + idx * step + ts_off;

    // Raw 8 bytes (little-endian dump as hex).
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG] %s idx=%zu  raw=%02x %02x %02x %02x "
                 "%02x %02x %02x %02x\n",
                 sample_labels[s], idx, ptr[0], ptr[1], ptr[2], ptr[3],
                 ptr[4], ptr[5], ptr[6], ptr[7]);

    // Interpretation A: bytes are a uint64 (e.g. PTP ns since epoch, or ns
    // since boot, or ns since scan start).
    uint64_t u64 = 0;
    std::memcpy(&u64, ptr, sizeof(uint64_t));
    ts_uint64[s] = u64;

    // Interpretation B: bytes are an IEEE-754 double encoded as seconds.
    double d_sec = 0.0;
    std::memcpy(&d_sec, ptr, sizeof(double));
    ts_double[s] = d_sec;

    // Interpretation C: bytes are an IEEE-754 double encoded as nanoseconds
    // (i.e. d_sec interpreted as ns directly).
    const double d_ns = d_sec;  // same memory, just rename for clarity.

    // Interpretation D: two uint32s (PTP layout: secs in low half, ns offset
    // in high half, or vice versa).
    uint32_t u32_lo = 0, u32_hi = 0;
    std::memcpy(&u32_lo, ptr, sizeof(uint32_t));
    std::memcpy(&u32_hi, ptr + 4, sizeof(uint32_t));

    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]              as_uint64=%-20lu  "
                 "as_double_sec=%.9f  as_double_ns=%.3e  "
                 "as_uint32_pair=(lo=%-10u hi=%-10u)\n",
                 static_cast<unsigned long>(u64), d_sec, d_ns,
                 u32_lo, u32_hi);
  }

  // Deltas between adjacent samples in each interpretation, to make collapse
  // obvious at a glance.
  const int64_t d_u64_01 =
      static_cast<int64_t>(ts_uint64[1]) - static_cast<int64_t>(ts_uint64[0]);
  const int64_t d_u64_0N =
      static_cast<int64_t>(ts_uint64[2]) - static_cast<int64_t>(ts_uint64[0]);
  const double  d_dbl_01 = ts_double[1] - ts_double[0];
  const double  d_dbl_0N = ts_double[2] - ts_double[0];

  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] deltas (mid - first / last - first):\n");
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG]   as_uint64_ns:    mid-first=%-15ld  "
               "last-first=%-15ld\n",
               static_cast<long>(d_u64_01), static_cast<long>(d_u64_0N));
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG]   as_double_sec:   mid-first=%.9f  "
               "last-first=%.9f\n",
               d_dbl_01, d_dbl_0N);

  // Heuristic verdict: order-of-magnitude check on each interpretation,
  // with the assumption that a healthy 10 Hz LiDAR scan should span ~0.1 s.
  // This is just a hint; the developer reads the raw lines above to confirm.
  auto plausible_seconds = [](double x) {
    return x > 1e-4 && x < 1.0;  // within 0.1 ms to 1 s
  };
  auto plausible_ns_as_uint64 = [](int64_t x) {
    return x > 100000 && x < 1000000000;  // 0.1 ms to 1 s, in ns
  };
  std::fprintf(stderr,
               "[LUMINAR_TS_DIAG] verdict (heuristic; check raw lines to confirm):\n");
  if (plausible_ns_as_uint64(d_u64_0N) && !plausible_seconds(d_dbl_0N)) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   uint64 ns interpretation looks plausible "
                 "(span %ld ns ~= %.3f ms)\n",
                 static_cast<long>(d_u64_0N), d_u64_0N * 1e-6);
  } else if (plausible_seconds(d_dbl_0N) && !plausible_ns_as_uint64(d_u64_0N)) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   FLOAT64 seconds interpretation looks "
                 "plausible (span %.6f s ~= %.3f ms). Current code memcpys as "
                 "uint64, which scrambles this.  Read as double instead.\n",
                 d_dbl_0N, d_dbl_0N * 1000.0);
  } else if (d_u64_0N == 0 && d_dbl_0N == 0.0) {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   timestamps appear COLLAPSED (zero span "
                 "in both interpretations). Driver likely fills every point "
                 "with the same scan-level stamp.\n");
  } else {
    std::fprintf(stderr,
                 "[LUMINAR_TS_DIAG]   verdict unclear -- see raw lines above. "
                 "Possible: per-scan timestamps with random jitter, or a "
                 "format we don't recognise.\n");
  }
  std::fprintf(stderr, "[LUMINAR_TS_DIAG] END\n");
  std::fflush(stderr);
}

bool findTimeField(const sensor_msgs::msg::PointCloud2& msg, int& time_off,
                   uint8_t& time_datatype, int& time_count) {
  time_off = -1;
  time_datatype = 0;
  time_count = 0;
  for (const auto& f : msg.fields) {
    if (f.name == "t" || f.name == "time" || f.name == "time_stamp" || f.name == "timestamp") {
      time_off = static_cast<int>(f.offset);
      time_datatype = f.datatype;
      time_count = static_cast<int>(f.count);
      return true;
    }
  }
  return false;
}

// Luminar stores hardware-clock ns in the timestamp union slot (8 raw bytes, not IEEE double).
uint64_t luminarPointTimestampNs(const PointType& pt) {
  uint64_t ts = 0;
  std::memcpy(&ts, &pt.timestamp, sizeof(uint64_t));
  return ts;
}

void clearPointTimeUnion(PointType& pt) {
  const uint64_t zero = 0;
  std::memcpy(&pt.timestamp, &zero, sizeof(uint64_t));
}

// Decode a Luminar per-point ABSOLUTE epoch timestamp (uint64 ns) directly from
// PointCloud2 bytes. UINT8[8] is always raw epoch-ns; FLOAT64 is raw epoch-ns
// only under the explicit driver contract. Ordinary FLOAT64 is handled
// separately as scan-relative seconds.
//
// Only 8-byte raw carriers are accepted on this absolute path because
// mergeAuxClouds() leaves them unshifted and deskewPointcloud() anchors on
// (ts - primary_min). A 32-bit field (UINT32)
// cannot hold an absolute epoch (it wraps every ~4.29 s) -- it would be a
// scan-relative counter, which this absolute path would silently misinterpret
// (dropping the inter-scan offset between aux and primary). So UINT32 is
// intentionally REJECTED here: a Luminar driver emitting UINT32 is unsupported
// and degrades to "no per-point time" (rigid transform) rather than corrupting
// deskew. `bytes_avail` (the field's room within point_step) guards the 8-byte
// read against a malformed/short time field.
bool luminarUsesRawEpochCarrier(
    uint8_t datatype, int count, bool float64_time_is_epoch_ns) {
  return (datatype == sensor_msgs::msg::PointField::UINT8 && count == 8) ||
         (float64_time_is_epoch_ns &&
          datatype == sensor_msgs::msg::PointField::FLOAT64 && count == 1);
}

bool luminarUsesRelativeFloat64Carrier(
    uint8_t datatype, int count, bool float64_time_is_epoch_ns) {
  return !float64_time_is_epoch_ns &&
         datatype == sensor_msgs::msg::PointField::FLOAT64 && count == 1;
}

bool luminarCloudUsesRelativeFloat64(
    const sensor_msgs::msg::PointCloud2& msg, bool float64_time_is_epoch_ns) {
  int time_off = -1;
  uint8_t datatype = 0;
  int count = 0;
  return findTimeField(msg, time_off, datatype, count) &&
         time_off >= 0 &&
         static_cast<uint32_t>(time_off) + sizeof(double) <= msg.point_step &&
         luminarUsesRelativeFloat64Carrier(
             datatype, count, float64_time_is_epoch_ns);
}

// Sanity-check the explicit FLOAT64 carrier contract against the first cloud.
// A raw epoch-ns carrier is plausible only when the uint64 interpretation lies
// close to the header epoch and spans at most a sweep. Ordinary doubles are
// plausible only as finite, small scan-relative seconds. This catches the
// otherwise silent configuration inversion where both interpretations still
// produce finite numbers.
bool luminarFloat64TimeContractMatches(
    const sensor_msgs::msg::PointCloud2& msg, bool expect_raw_epoch_ns,
    std::string& detail) {
  int time_off = -1;
  uint8_t datatype = 0;
  int count = 0;
  if (!findTimeField(msg, time_off, datatype, count) ||
      datatype != sensor_msgs::msg::PointField::FLOAT64 || count != 1) {
    detail = "cloud does not advertise a FLOAT64[1] time field";
    return true;
  }
  if (msg.is_bigendian || time_off < 0 || msg.point_step == 0 ||
      static_cast<uint32_t>(time_off) + sizeof(uint64_t) > msg.point_step) {
    detail = "malformed or big-endian FLOAT64 time field";
    return false;
  }
  const size_t point_count = static_cast<size_t>(msg.width) * msg.height;
  const size_t required_bytes =
      point_count * static_cast<size_t>(msg.point_step);
  if (point_count == 0 || msg.data.size() < required_bytes) {
    detail = "empty or truncated FLOAT64 time payload";
    return false;
  }

  double min_double = std::numeric_limits<double>::infinity();
  double max_double = -std::numeric_limits<double>::infinity();
  uint64_t min_raw = std::numeric_limits<uint64_t>::max();
  uint64_t max_raw = 0;
  bool doubles_finite = true;
  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t* tp =
        msg.data.data() + i * msg.point_step + static_cast<size_t>(time_off);
    double as_double = 0.0;
    uint64_t as_raw = 0;
    std::memcpy(&as_double, tp, sizeof(as_double));
    std::memcpy(&as_raw, tp, sizeof(as_raw));
    doubles_finite = doubles_finite && std::isfinite(as_double);
    min_double = std::min(min_double, as_double);
    max_double = std::max(max_double, as_double);
    if (as_raw != 0) {
      min_raw = std::min(min_raw, as_raw);
      max_raw = std::max(max_raw, as_raw);
    }
  }

  const uint64_t header_ns =
      static_cast<uint64_t>(msg.header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(msg.header.stamp.nanosec);
  const bool raw_nonempty = min_raw != std::numeric_limits<uint64_t>::max();
  const uint64_t raw_mid = raw_nonempty ? min_raw + (max_raw - min_raw) / 2 : 0;
  const uint64_t raw_header_error =
      raw_mid > header_ns ? raw_mid - header_ns : header_ns - raw_mid;
  const bool raw_plausible =
      raw_nonempty && header_ns > 1000000000000000ULL &&
      min_raw > 1000000000000000ULL && max_raw >= min_raw &&
      max_raw - min_raw <= 2000000000ULL &&
      raw_header_error <= 5000000000ULL;
  const bool relative_plausible =
      doubles_finite && min_double >= -1.0 && max_double <= 10.0 &&
      max_double >= min_double && max_double - min_double <= 2.0;

  std::ostringstream oss;
  oss << std::setprecision(9)
      << "double_range=[" << min_double << "," << max_double << "] "
      << "raw_range=[" << min_raw << "," << max_raw << "] "
      << "header_ns=" << header_ns
      << " relative_plausible=" << relative_plausible
      << " raw_epoch_plausible=" << raw_plausible;
  detail = oss.str();
  return expect_raw_epoch_ns ? raw_plausible
                             : (relative_plausible && !raw_plausible);
}

bool luminarRawTimestampNsFromBytes(
    const uint8_t* tp, uint8_t datatype, int count, size_t bytes_avail,
    bool float64_time_is_epoch_ns, uint64_t& out) {
  if (luminarUsesRawEpochCarrier(
          datatype, count, float64_time_is_epoch_ns) &&
      bytes_avail >= sizeof(uint64_t)) {
    std::memcpy(&out, tp, sizeof(uint64_t));
    return true;
  }
  return false;
}

// Decode a cloud's absolute point-time range (min/max over all points) once.
// This is the authoritative sweep identity for Luminar matching: header time
// is acquisition phase and must not gate the merge. Fails closed (invalid
// range) on any malformed point, short buffer, or unsupported time encoding.
gicp_localizer::LuminarTimestampRangeNs luminarTimestampRangeFromCloud(
    const sensor_msgs::msg::PointCloud2& msg,
    bool float64_time_is_epoch_ns) {
  gicp_localizer::LuminarTimestampRangeNs range;

  // The decode assumes little-endian payloads; a big-endian cloud would yield
  // garbage ranges that could still pass the gate by chance. Fail closed.
  if (msg.is_bigendian) {
    return range;
  }

  int time_off = -1;
  uint8_t time_datatype = 0;
  int time_count = 0;
  if (!findTimeField(msg, time_off, time_datatype, time_count) ||
      time_off < 0 || msg.point_step == 0 ||
      static_cast<uint32_t>(time_off) >= msg.point_step) {
    return range;
  }

  const size_t point_count = static_cast<size_t>(msg.width) * msg.height;
  const size_t required_bytes = point_count * static_cast<size_t>(msg.point_step);
  if (point_count == 0 || msg.data.size() < required_bytes) {
    return range;
  }

  const size_t bytes_avail = msg.point_step - static_cast<uint32_t>(time_off);
  range.min_ns = std::numeric_limits<uint64_t>::max();
  for (size_t i = 0; i < point_count; ++i) {
    uint64_t timestamp_ns = 0;
    if (!luminarRawTimestampNsFromBytes(
            msg.data.data() + i * msg.point_step + time_off,
            time_datatype, time_count, bytes_avail,
            float64_time_is_epoch_ns, timestamp_ns)) {
      return gicp_localizer::LuminarTimestampRangeNs{};
    }
    // [P3 FIX 2026-07-14] A zero per-point timestamp is the "no valid time"
    // sentinel (GLIM's matcher already skips ts==0). Including it here pins
    // min_ns to 0, which poisons the deskew anchor (a garbage arc), breaks the
    // aux watermark, and silently unmatches every aux. Skip it; range.valid
    // stays false only if EVERY point was zero.
    if (timestamp_ns == 0) continue;
    range.min_ns = std::min(range.min_ns, timestamp_ns);
    range.max_ns = std::max(range.max_ns, timestamp_ns);
    ++range.count;
  }
  range.valid = range.count > 0;
  return range;
}

// Copy per-point time from PointCloud2 into the gicp_localizer::Point union for the configured sensor.
// `point_step` bounds the field read so a malformed/short time field cannot read past the point.
void copyPointTimeFromCloud(const uint8_t* src, int time_off, uint8_t time_datatype, int time_count,
                           uint32_t point_step, gicp_localizer::SensorType sensor,
                           bool float64_time_is_epoch_ns, PointType& dst) {
  if (time_off < 0 || static_cast<uint32_t>(time_off) >= point_step) {
    return;
  }
  const uint8_t* tp = src + time_off;
  const size_t bytes_avail = point_step - static_cast<uint32_t>(time_off);

  switch (sensor) {
    case gicp_localizer::SensorType::LUMINAR: {
      uint64_t ts_raw = 0;
      if (luminarRawTimestampNsFromBytes(
              tp, time_datatype, time_count, bytes_avail,
              float64_time_is_epoch_ns, ts_raw)) {
        std::memcpy(&dst.timestamp, &ts_raw, sizeof(uint64_t));
        return;
      }
      if (luminarUsesRelativeFloat64Carrier(
              time_datatype, time_count, float64_time_is_epoch_ns) &&
          bytes_avail >= sizeof(double)) {
        double relative_seconds = 0.0;
        std::memcpy(&relative_seconds, tp, sizeof(double));
        if (std::isfinite(relative_seconds)) {
          dst.timestamp = relative_seconds;
        }
      }
      return;
    }
    case gicp_localizer::SensorType::OUSTER: {
      uint32_t t_ns = 0;
      switch (time_datatype) {
        case sensor_msgs::msg::PointField::UINT32:
          std::memcpy(&t_ns, tp, sizeof(uint32_t));
          break;
        case sensor_msgs::msg::PointField::FLOAT32: {
          float t_s = 0.f;
          std::memcpy(&t_s, tp, sizeof(float));
          t_ns = static_cast<uint32_t>(t_s * 1e9f);
          break;
        }
        case sensor_msgs::msg::PointField::FLOAT64: {
          double t_s = 0.;
          std::memcpy(&t_s, tp, sizeof(double));
          t_ns = static_cast<uint32_t>(t_s * 1e9);
          break;
        }
        default:
          break;
      }
      dst.t = t_ns;
      return;
    }
    case gicp_localizer::SensorType::VELODYNE: {
      float t_s = 0.f;
      switch (time_datatype) {
        case sensor_msgs::msg::PointField::FLOAT32:
          std::memcpy(&t_s, tp, sizeof(float));
          break;
        case sensor_msgs::msg::PointField::UINT32: {
          uint32_t t_ns = 0;
          std::memcpy(&t_ns, tp, sizeof(uint32_t));
          t_s = static_cast<float>(t_ns * 1e-9);
          break;
        }
        default:
          break;
      }
      dst.time = t_s;
      return;
    }
    case gicp_localizer::SensorType::HESAI: {
      double t_s = 0.;
      switch (time_datatype) {
        case sensor_msgs::msg::PointField::FLOAT64:
          std::memcpy(&t_s, tp, sizeof(double));
          break;
        case sensor_msgs::msg::PointField::FLOAT32: {
          float t_f = 0.f;
          std::memcpy(&t_f, tp, sizeof(float));
          t_s = static_cast<double>(t_f);
          break;
        }
        default:
          break;
      }
      dst.timestamp = t_s;
      return;
    }
    case gicp_localizer::SensorType::LIVOX: {
      if (time_datatype == sensor_msgs::msg::PointField::UINT8 && time_count == 8) {
        uint64_t t_ns = 0;
        std::memcpy(&t_ns, tp, sizeof(uint64_t));
        dst.timestamp = static_cast<double>(t_ns);
      } else if (time_datatype == sensor_msgs::msg::PointField::UINT32) {
        uint32_t t_ns = 0;
        std::memcpy(&t_ns, tp, sizeof(uint32_t));
        dst.timestamp = static_cast<double>(t_ns);
      } else if (time_datatype == sensor_msgs::msg::PointField::FLOAT64) {
        std::memcpy(&dst.timestamp, tp, sizeof(double));
      } else if (time_datatype == sensor_msgs::msg::PointField::FLOAT32) {
        float t_f = 0.f;
        std::memcpy(&t_f, tp, sizeof(float));
        dst.timestamp = static_cast<double>(t_f);
      }
      return;
    }
    default:
      return;
  }
}

void logLuminarTimestampStats(size_t num_points, const pcl::PointCloud<PointType>& cloud,
                              size_t unique_ros_times, bool raw_epoch_ns) {
  if (cloud.points.empty()) {
    return;
  }
  if (!raw_epoch_ns) {
    double tmin = std::numeric_limits<double>::infinity();
    double tmax = -std::numeric_limits<double>::infinity();
    for (const auto& pt : cloud.points) {
      if (!std::isfinite(pt.timestamp)) continue;
      tmin = std::min(tmin, pt.timestamp);
      tmax = std::max(tmax, pt.timestamp);
    }
    std::fprintf(
        stderr,
        "[LUMINAR_DBG] %zu pts, %zu unique_ros_times, relative FLOAT64 "
        "span_s=%.9f (min=%.9f max=%.9f)\n",
        num_points, unique_ros_times,
        (std::isfinite(tmin) && std::isfinite(tmax)) ? tmax - tmin
                                                     : -1.0,
        tmin, tmax);
    std::fflush(stderr);
    return;
  }
  uint64_t tmin = std::numeric_limits<uint64_t>::max();
  uint64_t tmax = 0;
  for (const auto& pt : cloud.points) {
    const uint64_t ts = luminarPointTimestampNs(pt);
    tmin = std::min(tmin, ts);
    tmax = std::max(tmax, ts);
  }
  const size_t mid = cloud.points.size() / 2;
  const uint64_t ts0 = luminarPointTimestampNs(cloud.points.front());
  const uint64_t ts_mid = luminarPointTimestampNs(cloud.points[mid]);
  const uint64_t tsN = luminarPointTimestampNs(cloud.points.back());
  std::fprintf(stderr,
               "[LUMINAR_DBG] %zu pts, %zu unique_ros_times, ts0=%lu tsMid=%lu tsN=%lu "
               "span_ns=%ld (minmax_span=%ld)\n",
               num_points, unique_ros_times, static_cast<unsigned long>(ts0),
               static_cast<unsigned long>(ts_mid), static_cast<unsigned long>(tsN),
               static_cast<long>(static_cast<int64_t>(tsN) - static_cast<int64_t>(ts0)),
               static_cast<long>(static_cast<int64_t>(tmax) - static_cast<int64_t>(tmin)));
  std::fflush(stderr);
}

// Shift per-point timestamps by `dt` seconds to rebase an aux scan's per-point
// times from its own header.stamp onto the merged cloud's primary header.stamp.
//
// Whether to actually shift depends on the underlying encoding:
//   * SCAN-RELATIVE encodings (FLOAT32/FLOAT64 seconds-since-scan-start,
//     UINT32 nanoseconds-since-scan-start) -> ADD dt so the value reads as
//     "seconds since primary scan start".
//   * ABSOLUTE-EPOCH encodings (Luminar Iris uint64 PTP epoch ns) -> DO NOT
//     shift. The downstream deskewer subtracts the merged cloud's
//     header.stamp to get a scan-relative offset, which already gives the
//     right (T_aux - T_primary + intra-aux-offset) when the value is left
//     at its absolute capture time. Shifting an absolute time by dt would
//     double-count the inter-scan offset and corrupt deskew.
//
// Luminar timestamp format (Luminar Iris Data Output Specification v1.3.0):
//   - The sensor does NOT emit a single uint64 epoch-ns field. It carries
//     48-bit integer epoch SECONDS once per packet header (§2.1, "PTP
//     Timestamp - seconds", UQ48.0) and a 32-bit SUB-SECOND NANOSECOND
//     count per ray (§2.2 / §2.6.3, "PTP Timestamp - nanoseconds", UQ32.0)
//     that wraps every 1 s. All fields are little-endian (§2).
//   - The uint64 epoch-ns this code reads is the upstream ROS driver's
//     reconstruction = header_seconds*1e9 + ray_nanoseconds. Correctness
//     depends on the driver performing that combination; a driver that
//     forwarded the bare 32-bit ns (sub-second sawtooth) would break deskew
//     across each 1 s rollover. Verify against the actual Luminar driver,
//     not the datasheet. (The "epoch time" guidance lives in the PTP
//     sections of the Product Information Guide, not the data layout.)
//
// `luminar_uint64=true` forces the 8 bytes at the time field to be read as
// uint64 regardless of declared datatype, because Luminar publishes the raw
// uint64 bits even when the field is mislabelled FLOAT64; generic FP
// arithmetic on those bits would scramble them.
void shiftCloudTimestamps(uint8_t* data, size_t num_points, uint32_t point_step,
                          int time_off, uint8_t time_datatype, int time_count,
                          double dt, bool luminar_uint64, double abs_clock_shift_s) {
  if (time_off < 0) return;
  // [P3 FIX 2026-07-10] Bounds: the widest carrier written below is 8 bytes;
  // a declared time field that does not fit its point would smear/overrun.
  if (static_cast<uint32_t>(time_off) + sizeof(uint64_t) > point_step) return;

  // Absolute-epoch path (Luminar Iris UINT8[8]): leave the per-point values
  // untouched. Each point already carries its absolute capture time; the
  // deskewer's t_i - merged_header.stamp computation in preprocessPointCloud
  // produces the correct intra-scan offset for both primary and aux points
  // without any rebasing here.
  if (luminar_uint64) {
    (void)dt;
    // P3 yaw-defect fix: absolute-epoch per-point times normally pass through
    // unshifted (each point carries its own capture time). But a CONSTANT
    // clock offset between this aux LiDAR and the primary/IMU clock makes
    // those absolute times land on the wrong segment of the IMU motion during
    // deskew — warping the merged scan during turns and creating false yaw
    // pressure. When a measured offset is configured
    // (lidar_concat/aux_time_offsets), correct the absolute times by it.
    // [REVIEW FIX 2026-07-08 P3] Both accepted absolute-epoch carriers hold
    // the same raw uint64 ns bits (see luminarRawTimestampNsFromBytes):
    // UINT8[8] (deployed Iris) AND a mislabelled FLOAT64 field. The offset
    // correction previously keyed on count == 8 only, silently skipping the
    // FLOAT64 carrier.
    if (abs_clock_shift_s != 0.0 &&
        (time_count == 8 ||
         time_datatype == sensor_msgs::msg::PointField::FLOAT64)) {
      const int64_t shift_ns = static_cast<int64_t>(abs_clock_shift_s * 1e9);
      for (size_t i = 0; i < num_points; ++i) {
        uint8_t* time_ptr = data + i * point_step + time_off;
        uint64_t val;
        std::memcpy(&val, time_ptr, sizeof(uint64_t));
        const int64_t shifted = static_cast<int64_t>(val) + shift_ns;
        val = static_cast<uint64_t>(std::max<int64_t>(0, shifted));
        std::memcpy(time_ptr, &val, sizeof(uint64_t));
      }
    }
    return;
  }

  for (size_t i = 0; i < num_points; ++i) {
    uint8_t* time_ptr = data + i * point_step + time_off;
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t val;
        std::memcpy(&val, time_ptr, sizeof(uint32_t));
        const int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
        val = static_cast<uint32_t>(std::max<int64_t>(0, shifted));
        std::memcpy(time_ptr, &val, sizeof(uint32_t));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT32: {
        float val;
        std::memcpy(&val, time_ptr, sizeof(float));
        val += static_cast<float>(dt);
        std::memcpy(time_ptr, &val, sizeof(float));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT64: {
        double val;
        std::memcpy(&val, time_ptr, sizeof(double));
        val += dt;
        std::memcpy(time_ptr, &val, sizeof(double));
        break;
      }
      case sensor_msgs::msg::PointField::UINT8: {
        // UINT8 count=8 == Luminar Iris uint64 PTP epoch nanoseconds
        // (driver reconstruction of header seconds + per-ray nanoseconds;
        // see this function's header comment block for the format and the
        // deskew-correctness argument). Values are absolute capture times --
        // leave them untouched. For any other UINT8 count (e.g. raw byte
        // runs that are not timestamps), there is nothing sensible to shift.
        break;
      }
      default:
        break;
    }
  }
}

}  // namespace gicp_localizer::detail
