#include "brightdrive_localization/health.hpp"

namespace brightdrive {

const char* health_state_name(HealthState state) {
  switch (state) {
    case HealthState::OK:
      return "OK";
    case HealthState::DEGRADED:
      return "DEGRADED";
    case HealthState::FAILED:
      return "FAILED";
  }
  return "FAILED";
}

HealthMonitor::HealthMonitor(const HealthParams& params) : params_(params) {}

bool HealthMonitor::bad(const HealthInput& input) const {
  if (!input.map_available) {
    return true;
  }
  if (input.scan_bad) {
    return true;
  }
  if (input.have_scan && input.scan_age > params_.scan_stale_s) {
    return true;
  }
  if (!input.have_scan && input.scan_age > params_.scan_stale_s) {
    return true;
  }
  return false;
}

bool HealthMonitor::good(const HealthInput& input) const {
  return input.map_available && input.scan_good && input.scan_age <= params_.scan_stale_s;
}

void HealthMonitor::transition(HealthState state, const std::string& reason,
                               HealthOutput& output) {
  if (state_ != state) {
    output.transitioned = true;
  }
  state_ = state;
  reason_ = reason;
  good_since_ = 0.0;
  bad_since_ = 0.0;
  have_good_since_ = false;
  have_bad_since_ = false;
}

HealthOutput HealthMonitor::update(const HealthInput& input) {
  HealthOutput output;
  output.state = state_;
  output.reason = reason_;

  const bool is_good = good(input);
  const bool is_bad = bad(input);

  if (is_good) {
    if (!have_good_since_ || !last_scan_good_) {
      good_since_ = input.now;
      have_good_since_ = true;
    }
    have_bad_since_ = false;
    last_scan_good_ = true;
    have_scan_state_ = true;
  } else if (is_bad) {
    if (!have_bad_since_ || last_scan_good_) {
      bad_since_ = input.now;
      have_bad_since_ = true;
    }
    have_good_since_ = false;
    last_scan_good_ = false;
    have_scan_state_ = true;
  }

  const double bad_duration = have_bad_since_ ? input.now - bad_since_ : 0.0;
  const double good_duration = have_good_since_ ? input.now - good_since_ : 0.0;

  if (!input.map_available) {
    transition(HealthState::FAILED, "map_unavailable", output);
  } else {
    switch (state_) {
      case HealthState::OK:
        if (input.scan_covariance_high) {
          transition(HealthState::DEGRADED, "map_covariance_high", output);
        } else if (input.gnss_age > params_.gnss_timeout_s) {
          transition(HealthState::DEGRADED, "gnss_timeout", output);
        } else if (bad_duration >= params_.degraded_after_bad_s) {
          transition(HealthState::DEGRADED, "scan_rejected", output);
        }
        break;
      case HealthState::DEGRADED:
        if (bad_duration >= params_.failed_after_bad_s) {
          transition(HealthState::FAILED, "scan_rejected", output);
        } else if (input.gnss_age > params_.sources_stale_s &&
                   input.imu_age > params_.sources_stale_s) {
          transition(HealthState::FAILED, "sources_stale", output);
        } else if (good_duration >= params_.recover_after_good_s &&
                   input.gnss_age <= params_.gnss_timeout_s && !input.scan_covariance_high) {
          transition(HealthState::OK, "recovered", output);
        }
        break;
      case HealthState::FAILED:
        if (input.relocalized && input.scan_good) {
          transition(HealthState::DEGRADED, "relocalized", output);
        }
        break;
    }
  }

  output.state = state_;
  output.reason = reason_;
  return output;
}

void HealthMonitor::set_state_for_test(HealthState state, const std::string& reason) {
  state_ = state;
  reason_ = reason;
}

}  // namespace brightdrive
