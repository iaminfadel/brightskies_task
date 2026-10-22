#include "brightdrive_localization/geodesy.hpp"

#include <cmath>

namespace brightdrive {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
}  // namespace

Eigen::Vector3d geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m) {
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  const double e2 = kFlattening * (2.0 - kFlattening);
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double normal = kSemiMajorAxis / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

  return Eigen::Vector3d{
      (normal + alt_m) * cos_lat * std::cos(lon),
      (normal + alt_m) * cos_lat * std::sin(lon),
      (normal * (1.0 - e2) + alt_m) * sin_lat};
}

Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d& ecef,
                            const Eigen::Vector3d& anchor_ecef,
                            double anchor_lat_deg,
                            double anchor_lon_deg) {
  const double lat = anchor_lat_deg * kDegToRad;
  const double lon = anchor_lon_deg * kDegToRad;
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);

  const Eigen::Vector3d delta = ecef - anchor_ecef;
  return Eigen::Vector3d{
      -sin_lon * delta.x() + cos_lon * delta.y(),
      -sin_lat * cos_lon * delta.x() - sin_lat * sin_lon * delta.y() + cos_lat * delta.z(),
      cos_lat * cos_lon * delta.x() + cos_lat * sin_lon * delta.y() + sin_lat * delta.z()};
}

Eigen::Vector3d geodetic_to_enu(double lat_deg, double lon_deg, double alt_m,
                                const GeoAnchor& anchor) {
  const Eigen::Vector3d anchor_ecef =
      geodetic_to_ecef(anchor.lat_deg, anchor.lon_deg, anchor.alt_m);
  return ecef_to_enu(geodetic_to_ecef(lat_deg, lon_deg, alt_m), anchor_ecef,
                     anchor.lat_deg, anchor.lon_deg);
}

double wrap_angle(double radians) {
  double wrapped = std::fmod(radians + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

}  // namespace brightdrive
