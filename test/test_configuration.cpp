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
  second.name = "second_board";
  second.hardware_parameters["device"] = "/dev/ttyUSB1";
  parsed.push_back(second);

  EXPECT_THROW(static_cast<void>(ssc32_ros2_control::inspect_hardware_resources(parsed)),
               std::invalid_argument);
  const auto configuration =
      ssc32_ros2_control::inspect_hardware_resources(parsed, std::string("second_board"));
  EXPECT_EQ(configuration.device, "/dev/ttyUSB1");

  const auto text = ssc32_ros2_control::render_configuration_text(configuration);
  EXPECT_NE(text.find("VALID SSC-32 configuration"), std::string::npos);
  EXPECT_NE(text.find("endpoint_pulses_us"), std::string::npos);
  const auto json = ssc32_ros2_control::render_configuration_json(configuration);
  EXPECT_NE(json.find("\"valid\": true"), std::string::npos);
  EXPECT_NE(json.find("\"hardware\": \"second_board\""), std::string::npos);
  EXPECT_NE(json.find("\"pulse_at_position_max_us\""), std::string::npos);
}

}  // namespace
