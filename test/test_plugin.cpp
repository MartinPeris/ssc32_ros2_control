// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

namespace {

TEST(Plugin, IsDiscoverableAndLoadable) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  const auto instance = loader.createSharedInstance("ssc32_ros2_control/Ssc32System");
  ASSERT_NE(instance, nullptr);
}

TEST(ExampleDescription, ParsesAsLyricalHardwareInfo) {
  const std::string path = std::string(SSC32_ROS2_CONTROL_SOURCE_DIR) + "/urdf/example.urdf";
  std::ifstream input(path);
  ASSERT_TRUE(input.is_open());
  const std::string urdf((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());

  const auto resources = hardware_interface::parse_control_resources_from_urdf(urdf);
  ASSERT_EQ(resources.size(), 1U);
  EXPECT_EQ(resources.front().name, "ssc32_example");
  EXPECT_TRUE(resources.front().is_async);
  EXPECT_EQ(resources.front().rw_rate, 50U);
  ASSERT_EQ(resources.front().joints.size(), 2U);
  EXPECT_EQ(resources.front().joints.front().parameters.at("channel"), "0");
}

}  // namespace
