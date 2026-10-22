// Time-based OK / DEGRADED / FAILED state machine from contract #9.
#pragma once

#include <string>

namespace brightdrive {

enum class HealthState : unsigned char { OK = 0, DEGRADED = 1, FAILED = 2 };

struct HealthParams {
  double degraded_after_bad_s = 0.6;
  double failed_after_bad_s = 2.0;
  double gnss_timeout_s = 1.0;
  double sources_stale_s = 2.0;
  double recover_after_good_s = 2.0;
  double scan_stale_s = 0.6;
  double covariance_trace_gate = 1.5;
};

struct HealthInput {
  double now = 0.0;
  bool have_scan = false;
  bool scan_good = false;
  bool scan_bad = false;
  double scan_age = 0.0;
  bool relocalized = false;
  double gnss_age = 0.0;
  double imu_age = 0.0;
  bool map_available = true;
  bool scan_covariance_high = false;
};

struct HealthOutput {
  HealthState state = HealthState::OK;
  std::string reason = "initializing";
  bool transitioned = false;
};

class HealthMonitor {
 public:
  explicit HealthMonitor(const HealthParams& params = HealthParams());

  HealthOutput update(const HealthInput& input);

  HealthState state() const { return state_; }
  const std::string& reason() const { return reason_; }

  // Test helpers.
  void set_state_for_test(HealthState state, const std::string& reason);

 private:
  bool bad(const HealthInput& input) const;
  bool good(const HealthInput& input) const;
  void transition(HealthState state, const std::string& reason, HealthOutput& output);

  HealthParams params_;
  HealthState state_ = HealthState::OK;
  std::string reason_ = "initializing";
  bool have_scan_state_ = false;
  bool last_scan_good_ = false;
  double good_since_ = 0.0;
  double bad_since_ = 0.0;
  bool have_good_since_ = false;
  bool have_bad_since_ = false;
};

const char* health_state_name(HealthState state);

}  // namespace brightdrive
