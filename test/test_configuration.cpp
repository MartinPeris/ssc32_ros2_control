// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>

#include "hardware_interface/component_parser.hpp"
#include "ssc32_ros2_control/configuration.hpp"
#include "ssc32_ros2_control/inspection.hpp"

namespace {

std::string example_urdf() {
  const std::string path = std::string(SSC32_ROS2_CONTROL_SOURCE_DIR) + "/urdf/example.urdf";
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open example URDF");
  }
  return std::string((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

std::vector<hardware_interface::HardwareInfo> resources() {
  return hardware_interface::parse_control_resources_from_urdf(example_urdf());
}

TEST(Configuration, ParsesExampleThroughSharedModel) {
  const auto configuration = ssc32_ros2_control::inspect_hardware_resources(resources());
  EXPECT_EQ(configuration.name, "ssc32_example");
  EXPECT_EQ(configuration.device, "/dev/ttyUSB0");
  EXPECT_EQ(configuration.baud_rate, 115200U);
  EXPECT_EQ(configuration.timeout, std::chrono::milliseconds(250));
  EXPECT_EQ(configuration.diagnostics_stale_after, std::chrono::milliseconds(1000));
  ASSERT_EQ(configuration.joints.size(), 2U);
  EXPECT_EQ(configuration.joints[0].name, "base_joint");
  EXPECT_EQ(configuration.joints[0].position_to_pulse(0.2), 1600U);
  EXPECT_EQ(configuration.joints[1].position_to_pulse(0.1), 1450U);
}

TEST(Configuration, AppliesExplicitDiagnosticsStalenessThreshold) {
  auto parsed = resources();
  ASSERT_EQ(parsed.size(), 1U);
  parsed.front().hardware_parameters["diagnostics_stale_after_ms"] = "2750";
  const auto configuration = ssc32_ros2_control::parse_hardware_configuration(parsed.front());
  EXPECT_EQ(configuration.diagnostics_stale_after, std::chrono::milliseconds(2750));
}

TEST(Configuration, RejectsDuplicateChannelsForPluginAndInspector) {
  auto parsed = resources();
  ASSERT_EQ(parsed.size(), 1U);
  ASSERT_EQ(parsed.front().joints.size(), 2U);
  parsed.front().joints[1].parameters["channel"] = "0";
  EXPECT_THROW(static_cast<void>(ssc32_ros2_control::parse_hardware_configuration(parsed.front())),
               std::invalid_argument);
}

TEST(Configuration, SelectsNamedHardwareAndRendersMachineReadableOutput) {
  auto parsed = resources();
  ASSERT_EQ(parsed.size(), 1U);
  auto second = parsed.front();
  second.name = "second_board_アーム";
  second.hardware_parameters["device"] = "/dev/ttyUSB1";
  parsed.push_back(second);

  EXPECT_THROW(static_cast<void>(ssc32_ros2_control::inspect_hardware_resources(parsed)),
               std::invalid_argument);
  const auto configuration =
      ssc32_ros2_control::inspect_hardware_resources(parsed, std::string("second_board_アーム"));
  EXPECT_EQ(configuration.device, "/dev/ttyUSB1");

  const auto text = ssc32_ros2_control::render_configuration_text(configuration);
  EXPECT_NE(text.find("VALID SSC-32 configuration"), std::string::npos);
  EXPECT_NE(text.find("endpoint_pulses_us"), std::string::npos);
  const auto json = ssc32_ros2_control::render_configuration_json(configuration);
  EXPECT_NE(json.find("\"valid\": true"), std::string::npos);
  EXPECT_NE(json.find("\"hardware\": \"second_board_アーム\""), std::string::npos);
  EXPECT_NE(json.find("\"pulse_at_position_max_us\""), std::string::npos);
}

TEST(Configuration, EscapesMachineReadableStringsAndReportsMissingHardware) {
  auto parsed = resources();
  ASSERT_EQ(parsed.size(), 1U);
  const std::string special = std::string("quote\" slash\\ back\b form\f line\n") +
                              "return\r tab\t control" + static_cast<char>(1);
  parsed.front().name = special;
  parsed.front().hardware_parameters["device"] = special;
  parsed.front().joints.front().name = special;
  const auto configuration = ssc32_ros2_control::inspect_hardware_resources(parsed, special);
  const auto json = ssc32_ros2_control::render_configuration_json(configuration);
  EXPECT_NE(json.find("quote\\\""), std::string::npos);
  EXPECT_NE(json.find("slash\\\\"), std::string::npos);
  EXPECT_NE(json.find("back\\b"), std::string::npos);
  EXPECT_NE(json.find("form\\f"), std::string::npos);
  EXPECT_NE(json.find("line\\n"), std::string::npos);
  EXPECT_NE(json.find("return\\r"), std::string::npos);
  EXPECT_NE(json.find("tab\\t"), std::string::npos);
  EXPECT_NE(json.find("control\\u0001"), std::string::npos);

  parsed.front().hardware_plugin_name = "other/Plugin";
  EXPECT_THROW(static_cast<void>(ssc32_ros2_control::inspect_hardware_resources(parsed)),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(
                   ssc32_ros2_control::inspect_hardware_resources(parsed, std::string("missing"))),
               std::invalid_argument);
}

TEST(Configuration, RejectsMalformedHardwareAndJointParameters) {
  auto expect_invalid = [](hardware_interface::HardwareInfo info) {
    EXPECT_THROW(static_cast<void>(ssc32_ros2_control::parse_hardware_configuration(info)),
                 std::exception);
  };

  auto info = resources().front();
  info.hardware_parameters.erase("device");
  expect_invalid(info);
  info = resources().front();
  info.hardware_parameters["baud_rate"] = "115200junk";
  expect_invalid(info);
  info = resources().front();
  info.hardware_parameters["timeout_ms"] = "0";
  expect_invalid(info);
  info = resources().front();
  info.hardware_parameters["diagnostics_stale_after_ms"] = "600001";
  expect_invalid(info);
  info = resources().front();
  info.joints.clear();
  expect_invalid(info);
  info = resources().front();
  info.joints.front().command_interfaces.front().name = "velocity";
  expect_invalid(info);
  info = resources().front();
  info.joints.front().state_interfaces.front().data_type = "float";
  expect_invalid(info);
  info = resources().front();
  info.joints.front().command_interfaces.front().min.clear();
  expect_invalid(info);
  info = resources().front();
  info.joints.front().parameters.erase("channel");
  expect_invalid(info);
  info = resources().front();
  info.joints.front().parameters["pulse_us_per_unit"] = "not-a-number";
  expect_invalid(info);
  info = resources().front();
  info.joints.front().parameters["max_speed_us_per_second"] = "0";
  expect_invalid(info);
}

}  // namespace
