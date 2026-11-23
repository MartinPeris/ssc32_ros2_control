// SPDX-License-Identifier: BSD-3-Clause
#include "ssc32_ros2_control/diagnostics.hpp"

#include <algorithm>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace ssc32_ros2_control {
Diagnostics::Diagnostics(const rclcpp::Node::SharedPtr& node,
                         const HardwareConfiguration& configuration)
    : configuration_(configuration) {
  if (node) {
    updater_ = std::make_unique<diagnostic_updater::Updater>(node);
    updater_->setHardwareID("ssc32:" + configuration_.device);
    updater_->add("Communication", this, &Diagnostics::report);
  }
}

void Diagnostics::mark_configured(const std::string& firmware) {
  {
    std::lock_guard<std::mutex> lock(details_mutex_);
    firmware_ = firmware;
    last_error_operation_.clear();
    last_error_.clear();
  }
  state_ = State::kConfiguredInactive;
}

void Diagnostics::mark_active() { state_ = State::kActive; }

void Diagnostics::mark_inactive() {
  if (state_.load() != State::kError) {
    state_ = State::kConfiguredInactive;
  }
}

void Diagnostics::mark_unconfigured() {
  {
    std::lock_guard<std::mutex> lock(details_mutex_);
    firmware_.clear();
    last_error_operation_.clear();
    last_error_.clear();
  }
  state_ = State::kUnconfigured;
}

void Diagnostics::record_read_success(const std::chrono::nanoseconds latency) noexcept {
  ++successful_reads_;
  const auto latency_count = latency.count();
  last_read_latency_ns_ = latency_count;
  update_max(maximum_read_latency_ns_, latency_count);
  last_read_ns_ = steady_now_ns();
}

void Diagnostics::record_write_success(const std::chrono::nanoseconds latency,
                                       const bool transmitted) noexcept {
  ++successful_write_cycles_;
  if (transmitted) {
    ++transmitted_writes_;
  } else {
    ++suppressed_writes_;
  }
  const auto latency_count = latency.count();
  last_write_latency_ns_ = latency_count;
  update_max(maximum_write_latency_ns_, latency_count);
}

void Diagnostics::record_error(const std::string& operation, const std::string& message) noexcept {
  if (operation == "read") {
    ++read_errors_;
  } else if (operation == "write") {
    ++write_errors_;
  } else {
    ++connection_errors_;
  }
  try {
    std::lock_guard<std::mutex> lock(details_mutex_);
    last_error_operation_ = operation;
    last_error_ = message;
  } catch (...) {
    // Diagnostics must not obscure the original hardware failure.
  }
  state_ = State::kError;
}

void Diagnostics::ensure_error(const std::string& operation, const std::string& message) noexcept {
  if (state_.load() != State::kError) {
    record_error(operation, message);
  }
}

void Diagnostics::report(diagnostic_updater::DiagnosticStatusWrapper& status) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto state = state_.load();
  const auto last_read = last_read_ns_.load();
  const auto age = last_read == 0 ? 0 : std::max<std::int64_t>(0, steady_now_ns() - last_read);

  std::string firmware;
  std::string error_operation;
  std::string error;
  {
    std::lock_guard<std::mutex> lock(details_mutex_);
    firmware = firmware_;
    error_operation = last_error_operation_;
    error = last_error_;
  }

  switch (state) {
    case State::kUnconfigured:
      status.summary(DiagnosticStatus::WARN, "Hardware is not configured");
      break;
    case State::kConfiguredInactive:
      status.summary(DiagnosticStatus::OK, "Connected; hardware is inactive");
      break;
    case State::kActive:
      if (last_read == 0) {
        status.summary(DiagnosticStatus::WARN, "Waiting for first successful read");
      } else if (age > std::chrono::duration_cast<std::chrono::nanoseconds>(
                           configuration_.diagnostics_stale_after)
                           .count()) {
        status.summary(DiagnosticStatus::STALE, "No recent successful read");
      } else {
        status.summary(DiagnosticStatus::OK, "Communication healthy");
      }
      break;
    case State::kError:
      status.summary(DiagnosticStatus::ERROR,
                     error.empty() ? "Hardware communication failed" : error);
      break;
  }

  status.add("state", state_name(state));
  status.add("device", configuration_.device);
  status.add("baud_rate", configuration_.baud_rate);
  status.add("joint_count", configuration_.joints.size());
  status.add("firmware", firmware.empty() ? "unknown" : firmware);
  status.add("stale_after_ms", configuration_.diagnostics_stale_after.count());
  status.add("successful_reads", successful_reads_.load());
  status.add("successful_write_cycles", successful_write_cycles_.load());
  status.add("transmitted_writes", transmitted_writes_.load());
  status.add("suppressed_writes", suppressed_writes_.load());
  status.add("read_errors", read_errors_.load());
  status.add("write_errors", write_errors_.load());
  status.add("connection_errors", connection_errors_.load());
  status.add("last_read_age_ms", last_read == 0 ? -1.0 : milliseconds(age));
  status.add("last_read_latency_ms", milliseconds(last_read_latency_ns_.load()));
  status.add("maximum_read_latency_ms", milliseconds(maximum_read_latency_ns_.load()));
  status.add("last_write_latency_ms", milliseconds(last_write_latency_ns_.load()));
  status.add("maximum_write_latency_ms", milliseconds(maximum_write_latency_ns_.load()));
  status.add("last_error_operation", error_operation.empty() ? "none" : error_operation);
}

std::int64_t Diagnostics::steady_now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

const char* Diagnostics::state_name(const State state) noexcept {
  switch (state) {
    case State::kUnconfigured:
      return "unconfigured";
    case State::kConfiguredInactive:
      return "configured_inactive";
    case State::kActive:
      return "active";
    case State::kError:
      return "error";
  }
  return "unknown";
}

void Diagnostics::update_max(std::atomic<std::int64_t>& maximum,
                             const std::int64_t candidate) noexcept {
  auto current = maximum.load();
  while (candidate > current && !maximum.compare_exchange_weak(current, candidate)) {
  }
}

double Diagnostics::milliseconds(const std::int64_t nanoseconds) noexcept {
  return static_cast<double>(nanoseconds) / 1000000.0;
}

}  // namespace ssc32_ros2_control
