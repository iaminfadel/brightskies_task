#include <cmath>

#include <gtest/gtest.h>

#include "brightdrive_localization/geodesy.hpp"

namespace brightdrive {
namespace {

constexpr double kDegToRad = M_PI / 180.0;

TEST(Geodesy, AnchorIsTheOrigin) {
  const GeoAnchor anchor{0.0, 0.0, 0.0};
  const Eigen::Vector3d enu = geodetic_to_enu(anchor.lat_deg, anchor.lon_deg, anchor.alt_m, anchor);
  EXPECT_LT(enu.norm(), 1e-6);
}

TEST(Geodesy, EastAndUpAreMetric) {
  const GeoAnchor anchor{0.0, 0.0, 0.0};
  const double meters = 25.0;
  const double dlon = meters / (kSemiMajorAxis * std::cos(anchor.lat_deg * kDegToRad)) / kDegToRad;

  const Eigen::Vector3d east = geodetic_to_enu(anchor.lat_deg, anchor.lon_deg + dlon, 0.0, anchor);
  EXPECT_NEAR(east.x(), meters, 1e-3);
  EXPECT_NEAR(east.y(), 0.0, 1e-3);
  EXPECT_NEAR(east.z(), 0.0, 1e-3);

  const Eigen::Vector3d up = geodetic_to_enu(anchor.lat_deg, anchor.lon_deg, 12.5, anchor);
  EXPECT_NEAR(up.z(), 12.5, 1e-6);
}

TEST(Geodesy, NearbyRoundTripIsSubMillimetre) {
  const GeoAnchor anchor{51.5, -0.12, 35.0};
  const double dlat = 1e-6;
  const double dlon = -2e-6;
  const Eigen::Vector3d a = geodetic_to_enu(anchor.lat_deg, anchor.lon_deg, anchor.alt_m, anchor);
  const Eigen::Vector3d b =
      geodetic_to_enu(anchor.lat_deg + dlat, anchor.lon_deg + dlon, anchor.alt_m + 4.0, anchor);
  const Eigen::Vector3d delta = b - a;

  const double north_per_deg = 111132.92 - 559.82 * std::cos(2.0 * anchor.lat_deg * kDegToRad);
  const double east_per_deg = 111412.84 * std::cos(anchor.lat_deg * kDegToRad) -
                              93.5 * std::cos(3.0 * anchor.lat_deg * kDegToRad);
  EXPECT_NEAR(delta.y(), dlat * north_per_deg, 1e-3);
  EXPECT_NEAR(delta.x(), dlon * east_per_deg, 1e-3);
  EXPECT_NEAR(delta.z(), 4.0, 1e-3);
}

TEST(Geodesy, YawWrapsToMinusPiPi) {
  EXPECT_NEAR(wrap_angle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrap_angle(M_PI + 0.1), -M_PI + 0.1, 1e-12);
  EXPECT_NEAR(wrap_angle(-M_PI - 0.1), M_PI - 0.1, 1e-12);
  EXPECT_NEAR(wrap_angle(4.0 * M_PI), 0.0, 1e-12);
  EXPECT_LT(std::abs(wrap_angle(1000.0)), M_PI);
}

}  // namespace
}  // namespace brightdrive
