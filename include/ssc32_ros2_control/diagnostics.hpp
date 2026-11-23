// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/node.hpp"
#include "ssc32_ros2_control/configuration.hpp"

namespace ssc32_ros2_control {

class Diagnostics final {
 public:
  Diagnostics(const rclcpp::Node::SharedPtr& node, const HardwareConfiguration& configuration);

  void mark_configured(const std::string& firmware);
  void mark_active();
  void mark_inactive();
  void mark_unconfigured();
  void record_read_success(std::chrono::nanoseconds latency) noexcept;
  void record_write_success(std::chrono::nanoseconds latency, bool transmitted) noexcept;
  void record_error(const std::string& operation, const std::string& message) noexcept;
  void ensure_error(const std::string& operation, const std::string& message) noexcept;

  void report(diagnostic_updater::DiagnosticStatusWrapper& status);

 private:
  enum class State : std::uint8_t { kUnconfigured, kConfiguredInactive, kActive, kError };

  static std::int64_t steady_now_ns() noexcept;
  static const char* state_name(State state) noexcept;
  static void update_max(std::atomic<std::int64_t>& maximum, std::int64_t candidate) noexcept;
  static double milliseconds(std::int64_t nanoseconds) noexcept;

  HardwareConfiguration configuration_;
  std::atomic<State> state_{State::kUnconfigured};
  std::atomic<std::uint64_t> successful_reads_{0U};
  std::atomic<std::uint64_t> successful_write_cycles_{0U};
  std::atomic<std::uint64_t> transmitted_writes_{0U};
  std::atomic<std::uint64_t> suppressed_writes_{0U};
  std::atomic<std::uint64_t> read_errors_{0U};
  std::atomic<std::uint64_t> write_errors_{0U};
  std::atomic<std::uint64_t> connection_errors_{0U};
  std::atomic<std::int64_t> last_read_ns_{0};
  std::atomic<std::int64_t> last_read_latency_ns_{0};
  std::atomic<std::int64_t> maximum_read_latency_ns_{0};
  std::atomic<std::int64_t> last_write_latency_ns_{0};
  std::atomic<std::int64_t> maximum_write_latency_ns_{0};
  mutable std::mutex details_mutex_;
  std::string firmware_;
  std::string last_error_operation_;
  std::string last_error_;
  // Declared last so the timer and its callback are destroyed before snapshot state.
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

}  // namespace ssc32_ros2_control
