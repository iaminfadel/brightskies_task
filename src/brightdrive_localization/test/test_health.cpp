#include <gtest/gtest.h>

#include "brightdrive_localization/health.hpp"

namespace brightdrive {
namespace {

HealthInput scan_input(double now, bool good, bool bad) {
  HealthInput input;
  input.now = now;
  input.have_scan = true;
  input.scan_good = good;
  input.scan_bad = bad;
  input.scan_age = 0.1;
  input.gnss_age = 0.1;
  input.imu_age = 0.05;
  input.map_available = true;
  return input;
}

TEST(Health, StartsOk) {
  HealthMonitor monitor;
  const HealthOutput output = monitor.update(scan_input(0.0, true, false));
  EXPECT_EQ(output.state, HealthState::OK);
}

TEST(Health, BadScansDegradeAfterSixTenths) {
  HealthMonitor monitor;
  monitor.update(scan_input(0.0, true, false));
  HealthOutput output;
  for (int i = 1; i <= 3; ++i) {
    output = monitor.update(scan_input(0.2 * i, false, true));
  }
  EXPECT_EQ(output.state, HealthState::OK);
  output = monitor.update(scan_input(0.8, false, true));
  EXPECT_EQ(output.state, HealthState::DEGRADED);
  EXPECT_EQ(output.reason, "scan_rejected");
  EXPECT_TRUE(output.transitioned);
}

TEST(Health, BadScansFailAfterTwoSeconds) {
  HealthMonitor monitor;
  monitor.update(scan_input(0.0, true, false));
  HealthOutput output;
  for (int i = 1; i <= 10; ++i) {
    output = monitor.update(scan_input(0.2 * i, false, true));
  }
  EXPECT_EQ(output.state, HealthState::DEGRADED);
  for (int i = 11; i <= 20; ++i) {
    output = monitor.update(scan_input(0.2 * i, false, true));
  }
  EXPECT_EQ(output.state, HealthState::FAILED);
  EXPECT_EQ(output.reason, "scan_rejected");
}

TEST(Health, GoodScansRecoverAfterTwoSeconds) {
  HealthMonitor monitor;
  monitor.update(scan_input(0.0, true, false));
  for (int i = 1; i <= 4; ++i) {
    monitor.update(scan_input(0.2 * i, false, true));
  }
  ASSERT_EQ(monitor.state(), HealthState::DEGRADED);
  for (int i = 5; i <= 15; ++i) {
    monitor.update(scan_input(0.2 * i, true, false));
  }
  EXPECT_EQ(monitor.state(), HealthState::OK);
}

TEST(Health, GnssTimeoutDegradesImmediately) {
  HealthMonitor monitor;
  HealthInput input = scan_input(1.0, true, false);
  input.gnss_age = 1.05;
  const HealthOutput output = monitor.update(input);
  EXPECT_EQ(output.state, HealthState::DEGRADED);
  EXPECT_EQ(output.reason, "gnss_timeout");
}

TEST(Health, SourcesStaleFail) {
  HealthMonitor monitor;
  monitor.update(scan_input(0.0, true, false));
  HealthInput input = scan_input(1.0, true, false);
  input.gnss_age = 1.2;
  input.imu_age = 1.2;
  ASSERT_EQ(monitor.update(input).state, HealthState::DEGRADED);
  input.now = 1.2;
  input.gnss_age = 2.5;
  input.imu_age = 2.5;
  const HealthOutput output = monitor.update(input);
  EXPECT_EQ(output.state, HealthState::FAILED);
  EXPECT_EQ(output.reason, "sources_stale");
}

TEST(Health, MapUnavailableFails) {
  HealthMonitor monitor;
  HealthInput input = scan_input(1.0, true, false);
  input.map_available = false;
  const HealthOutput output = monitor.update(input);
  EXPECT_EQ(output.state, HealthState::FAILED);
  EXPECT_EQ(output.reason, "map_unavailable");
}

TEST(Health, RelocalizationRecoversOnlyOneStep) {
  HealthMonitor monitor;
  monitor.set_state_for_test(HealthState::FAILED, "scan_rejected");
  HealthOutput output = monitor.update(scan_input(10.0, true, false));
  EXPECT_EQ(output.state, HealthState::FAILED);

  HealthInput input = scan_input(10.2, true, false);
  input.relocalized = true;
  output = monitor.update(input);
  EXPECT_EQ(output.state, HealthState::DEGRADED);
  EXPECT_EQ(output.reason, "relocalized");

  // The two-second good window starts now; one healthy scan is not enough.
  output = monitor.update(scan_input(10.4, true, false));
  EXPECT_EQ(output.state, HealthState::DEGRADED);
  for (int i = 0; i < 12; ++i) {
    output = monitor.update(scan_input(10.6 + 0.2 * i, true, false));
  }
  EXPECT_EQ(output.state, HealthState::OK);
}

TEST(Health, CovarianceSpikeDegrades) {
  HealthMonitor monitor;
  monitor.update(scan_input(0.0, true, false));
  HealthInput input = scan_input(0.2, true, false);
  input.scan_covariance_high = true;
  const HealthOutput output = monitor.update(input);
  EXPECT_EQ(output.state, HealthState::DEGRADED);
  EXPECT_EQ(output.reason, "map_covariance_high");
}

TEST(Health, StaleScanCountsAsBad) {
  HealthMonitor monitor;
  monitor.update(scan_input(0.0, true, false));
  HealthInput input = scan_input(0.7, false, false);
  input.scan_age = 0.9;
  monitor.update(input);
  input.now = 1.3;
  const HealthOutput output = monitor.update(input);
  EXPECT_EQ(output.state, HealthState::DEGRADED);
  EXPECT_EQ(output.reason, "scan_rejected");
}

}  // namespace
}  // namespace brightdrive
