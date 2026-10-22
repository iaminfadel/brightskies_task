// WGS84 -> ECEF -> local ENU conversion for the saved-map georeference.
#pragma once

#include <Eigen/Core>

namespace brightdrive {

struct GeoAnchor {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;
};

// WGS84 ellipsoid constants (contract #9).
constexpr double kSemiMajorAxis = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;

Eigen::Vector3d geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m);

Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d& ecef,
                            const Eigen::Vector3d& anchor_ecef,
                            double anchor_lat_deg,
                            double anchor_lon_deg);

// Position of a WGS84 fix in the saved map frame, metres.
Eigen::Vector3d geodetic_to_enu(double lat_deg, double lon_deg, double alt_m,
                                const GeoAnchor& anchor);

double wrap_angle(double radians);

}  // namespace brightdrive
