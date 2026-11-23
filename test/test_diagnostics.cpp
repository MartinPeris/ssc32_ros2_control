// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ssc32_ros2_control/diagnostics.hpp"

namespace {
using namespace std::chrono_literals;

ssc32_ros2_control::HardwareConfiguration configuration() {
  ssc32_ros2_control::HardwareConfiguration result;
  result.name = "test_board";
  result.device = "/dev/test-ssc32";
  result.baud_rate = 115200U;
  result.timeout = 100ms;
  result.diagnostics_stale_after = 10ms;
  result.joints.resize(2U);
  return result;
}

std::string value(const diagnostic_updater::DiagnosticStatusWrapper& status,
                  const std::string& key) {
  for (const auto& item : status.values) {
    if (item.key == key) {
      return item.value;
    }
  }
  return {};
}

class RclcppScope final {
 public:
  RclcppScope() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  ~RclcppScope() { rclcpp::shutdown(); }
};

TEST(Diagnostics, ReportsLifecycleHealthStalenessAndErrors) {
  RclcppScope rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("ssc32_diagnostics_state_test");
  ssc32_ros2_control::Diagnostics diagnostics(node, configuration());
  diagnostic_updater::DiagnosticStatusWrapper status;

  diagnostics.report(status);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(value(status, "state"), "unconfigured");

  diagnostics.mark_configured("SSC32-V2.03XE");
  diagnostics.mark_active();
  diagnostics.record_read_success(2ms);
  diagnostics.record_write_success(1ms, true);
  status.values.clear();
  diagnostics.report(status);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(value(status, "firmware"), "SSC32-V2.03XE");
  EXPECT_EQ(value(status, "transmitted_writes"), "1");

  std::this_thread::sleep_for(15ms);
  status.values.clear();
  diagnostics.report(status);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::STALE);

  diagnostics.record_error("read", "serial timeout");
  status.values.clear();
  diagnostics.report(status);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "serial timeout");
  EXPECT_EQ(value(status, "read_errors"), "1");
  EXPECT_EQ(value(status, "last_error_operation"), "read");
}

TEST(Diagnostics, PublishesOnStandardDiagnosticsTopic) {
  RclcppScope rclcpp_scope;
  auto hardware_node = std::make_shared<rclcpp::Node>("ssc32_diagnostics_publisher_test");
  auto monitor_node = std::make_shared<rclcpp::Node>("ssc32_diagnostics_monitor_test");
  ssc32_ros2_control::Diagnostics diagnostics(hardware_node, configuration());
  diagnostics.mark_configured("SSC32-V2.03XE");
  diagnostics.mark_active();
  diagnostics.record_read_success(2ms);

  std::mutex mutex;
  std::condition_variable condition;
  bool received = false;
  auto subscription = monitor_node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray& message) {
        for (const auto& status : message.status) {
          if (status.hardware_id == "ssc32:/dev/test-ssc32" &&
              status.name.find("Communication") != std::string::npos) {
            {
              std::lock_guard<std::mutex> lock(mutex);
              received = true;
            }
            condition.notify_all();
          }
        }
      });

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(hardware_node);
  executor->add_node(monitor_node);
  std::thread worker([&] { executor->spin(); });
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, 2s, [&] { return received; }));
  }
  executor->cancel();
  worker.join();
  static_cast<void>(subscription);
}

}  // namespace
