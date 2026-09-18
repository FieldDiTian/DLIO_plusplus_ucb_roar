#ifndef GICP_LOCALIZER__NAVSATFIX_HPP_
#define GICP_LOCALIZER__NAVSATFIX_HPP_

#include <cmath>
#include <limits>

namespace gicp_localizer::navsatfix {

struct EnuOrigin {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

// Convert a local ENU displacement from the configured datum to WGS84.
// ECEF is used internally so the published fix remains well behaved over a
// full race circuit rather than relying on a small-angle latitude/longitude
// approximation.
inline bool enuToWgs84(double east_m, double north_m, double up_m,
                       const EnuOrigin &origin, double &latitude_deg,
                       double &longitude_deg, double &altitude_m) {
  constexpr double kA = 6378137.0;
  constexpr double kE2 = 6.6943799901413165e-3;
  constexpr double kDegToRad = 0.017453292519943295;
  constexpr double kRadToDeg = 57.29577951308232;
  if (!std::isfinite(east_m) || !std::isfinite(north_m) ||
      !std::isfinite(up_m) || !std::isfinite(origin.latitude_deg) ||
      !std::isfinite(origin.longitude_deg) ||
      !std::isfinite(origin.altitude_m) || origin.latitude_deg < -90.0 ||
      origin.latitude_deg > 90.0 || origin.longitude_deg < -180.0 ||
      origin.longitude_deg > 180.0) {
    return false;
  }

  const double lat0 = origin.latitude_deg * kDegToRad;
  const double lon0 = origin.longitude_deg * kDegToRad;
  const double sin_lat0 = std::sin(lat0);
  const double cos_lat0 = std::cos(lat0);
  const double sin_lon0 = std::sin(lon0);
  const double cos_lon0 = std::cos(lon0);
  const double n0 = kA / std::sqrt(1.0 - kE2 * sin_lat0 * sin_lat0);
  const double x0 = (n0 + origin.altitude_m) * cos_lat0 * cos_lon0;
  const double y0 = (n0 + origin.altitude_m) * cos_lat0 * sin_lon0;
  const double z0 = (n0 * (1.0 - kE2) + origin.altitude_m) * sin_lat0;

  const double x = x0 - sin_lon0 * east_m - sin_lat0 * cos_lon0 * north_m +
                   cos_lat0 * cos_lon0 * up_m;
  const double y = y0 + cos_lon0 * east_m - sin_lat0 * sin_lon0 * north_m +
                   cos_lat0 * sin_lon0 * up_m;
  const double z = z0 + cos_lat0 * north_m + sin_lat0 * up_m;
  const double p = std::hypot(x, y);
  if (!std::isfinite(p) || !std::isfinite(z)) {
    return false;
  }

  double latitude = std::atan2(z, p * (1.0 - kE2));
  const double longitude = std::atan2(y, x);
  double height = 0.0;
  for (int i = 0; i < 10; ++i) {
    const double sin_lat = std::sin(latitude);
    const double n = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
    height = p / std::cos(latitude) - n;
    const double next = std::atan2(
        z, p * (1.0 - kE2 * n / (n + height)));
    if (std::abs(next - latitude) < 1e-13) {
      latitude = next;
      break;
    }
    latitude = next;
  }
  const double sin_lat = std::sin(latitude);
  const double n = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
  height = p / std::cos(latitude) - n;
  latitude_deg = latitude * kRadToDeg;
  longitude_deg = longitude * kRadToDeg;
  altitude_m = height;
  return std::isfinite(latitude_deg) && std::isfinite(longitude_deg) &&
         std::isfinite(altitude_m);
}

} // namespace gicp_localizer::navsatfix

#endif // GICP_LOCALIZER__NAVSATFIX_HPP_
