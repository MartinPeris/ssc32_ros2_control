// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ssc32_ros2_control/ssc32_system.hpp"
#include "support/ssc32_emulator.hpp"

namespace {

std::string example_urdf(const std::string& device) {
  const std::string path = std::string(SSC32_ROS2_CONTROL_SOURCE_DIR) + "/urdf/example.urdf";
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open example URDF");
  }
  std::string urdf((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const std::string placeholder = "/dev/ttyUSB0";
  const auto position = urdf.find(placeholder);
  if (position == std::string::npos) {
    throw std::runtime_error("example URDF has no device placeholder");
  }
  urdf.replace(position, placeholder.size(), device);
  return urdf;
}

hardware_interface::HardwareComponentInterfaceParams parameters_for(const std::string& device) {
  const auto resources =
      hardware_interface::parse_control_resources_from_urdf(example_urdf(device));
  if (resources.size() != 1U) {
    throw std::runtime_error("unexpected hardware resource count");
  }
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = resources.front();
  return params;
}

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class RclcppScope final {
 public:
  RclcppScope() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
  ~RclcppScope() { rclcpp::shutdown(); }
};

TEST(SystemPtyIntegration, ExercisesLifecycleAndInterfaces) {
  RclcppScope rclcpp_scope;
  ssc32_ros2_control::test::Ssc32Emulator emulator({{0U, 1500U}, {1U, 1500U}});
  ssc32_ros2_control::Ssc32System system;

  require(system.on_init(parameters_for(emulator.device())) ==
              hardware_interface::CallbackReturn::SUCCESS,
          "system initialization failed");
  const auto states = system.on_export_state_interfaces();
  const auto commands = system.on_export_command_interfaces();
  require(states.size() == 2U, "unexpected state interface count");
  require(commands.size() == 2U, "unexpected command interface count");

  const rclcpp_lifecycle::State state;
  require(system.on_configure(state) == hardware_interface::CallbackReturn::SUCCESS,
          "system configuration failed");
  require(system.on_activate(state) == hardware_interface::CallbackReturn::SUCCESS,
          "system activation failed");
  require(system.read(rclcpp::Time(0), rclcpp::Duration(0, 20000000)) ==
              hardware_interface::return_type::OK,
          "system read failed");
  bool command_set = false;
  for (const auto& command : commands) {
    if (command->get_name() == "base_joint/position") {
      command_set = command->set_value(0.1);
    }
  }
  require(command_set, "failed to set base_joint/position command interface");
  require(system.write(rclcpp::Time(0), rclcpp::Duration(0, 20000000)) ==
              hardware_interface::return_type::OK,
          "system write failed");
  require(emulator.wait_for_move_count(1U, std::chrono::seconds(1)),
          "emulator did not receive a move command");
  EXPECT_EQ(emulator.pulse(0U), 1550U);
  EXPECT_EQ(emulator.pulse(1U), 1500U);
  ASSERT_EQ(emulator.move_commands().size(), 1U);
  EXPECT_EQ(emulator.move_commands().front(), "#0 P1550 S100 #1 P1500 S75 \r");

  require(system.on_deactivate(state) == hardware_interface::CallbackReturn::SUCCESS,
          "system deactivation failed");
  require(system.on_cleanup(state) == hardware_interface::CallbackReturn::SUCCESS,
          "system cleanup failed");
  EXPECT_TRUE(emulator.error().empty()) << emulator.error();
}

TEST(SystemPtyIntegration, ReportsDroppedAndTruncatedResponses) {
  RclcppScope rclcpp_scope;
  ssc32_ros2_control::test::Ssc32Emulator emulator({{0U, 1500U}, {1U, 1500U}});
  ssc32_ros2_control::Ssc32System system;
  require(system.on_init(parameters_for(emulator.device())) ==
              hardware_interface::CallbackReturn::SUCCESS,
          "system initialization failed");
  require(system.on_export_state_interfaces().size() == 2U, "unexpected state interface count");
  require(system.on_export_command_interfaces().size() == 2U, "unexpected command interface count");
  const rclcpp_lifecycle::State state;
  require(system.on_configure(state) == hardware_interface::CallbackReturn::SUCCESS,
          "system configuration failed");

  emulator.fault_next_response(ssc32_ros2_control::test::Ssc32Emulator::ResponseFault::kDrop);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 20000000)),
            hardware_interface::return_type::ERROR);

  emulator.fault_next_response(ssc32_ros2_control::test::Ssc32Emulator::ResponseFault::kTruncate);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 20000000)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.on_cleanup(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(emulator.error().empty()) << emulator.error();
}

TEST(SystemPtyIntegration, RejectsDelayedFirmwareResponseDuringConfigure) {
  RclcppScope rclcpp_scope;
  ssc32_ros2_control::test::Ssc32Emulator emulator;
  emulator.set_response_delay(std::chrono::milliseconds(300));
  ssc32_ros2_control::Ssc32System system;
  require(system.on_init(parameters_for(emulator.device())) ==
              hardware_interface::CallbackReturn::SUCCESS,
          "system initialization failed");
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State()),
            hardware_interface::CallbackReturn::ERROR);
}

TEST(SystemPtyIntegration, ReportsDisconnectWithoutAttemptingRecoveryMotion) {
  RclcppScope rclcpp_scope;
  ssc32_ros2_control::test::Ssc32Emulator emulator({{0U, 1500U}, {1U, 1500U}});
  ssc32_ros2_control::Ssc32System system;
  require(system.on_init(parameters_for(emulator.device())) ==
              hardware_interface::CallbackReturn::SUCCESS,
          "system initialization failed");
  require(system.on_export_state_interfaces().size() == 2U, "unexpected state interface count");
  require(system.on_export_command_interfaces().size() == 2U, "unexpected command interface count");
  const rclcpp_lifecycle::State state;
  require(system.on_configure(state) == hardware_interface::CallbackReturn::SUCCESS,
          "system configuration failed");

  emulator.fault_next_response(ssc32_ros2_control::test::Ssc32Emulator::ResponseFault::kDisconnect);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 20000000)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.on_error(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(emulator.move_commands().empty());
  EXPECT_TRUE(emulator.error().empty()) << emulator.error();
}

TEST(SystemPtyIntegration, RejectsReportedPulseOutsideConfiguredEnvelope) {
  RclcppScope rclcpp_scope;
  ssc32_ros2_control::test::Ssc32Emulator emulator({{0U, 2500U}, {1U, 1500U}});
  ssc32_ros2_control::Ssc32System system;
  require(system.on_init(parameters_for(emulator.device())) ==
              hardware_interface::CallbackReturn::SUCCESS,
          "system initialization failed");
  require(system.on_export_state_interfaces().size() == 2U, "unexpected state interface count");
  require(system.on_export_command_interfaces().size() == 2U, "unexpected command interface count");
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State()),
            hardware_interface::CallbackReturn::ERROR);
  EXPECT_TRUE(emulator.move_commands().empty());
}

}  // namespace
