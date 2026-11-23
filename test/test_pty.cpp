// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "ssc32/serial_transport.hpp"
#include "ssc32_ros2_control/driver.hpp"
#include "support/ssc32_emulator.hpp"

namespace {

std::vector<ssc32_ros2_control::JointCalibration> calibrations() {
  ssc32_ros2_control::JointCalibration base;
  base.name = "base";
  base.channel = 0;
  base.zero_pulse_us = 1500;
  base.pulse_us_per_unit = 500.0;
  base.direction = 1;
  base.min_pulse_us = 1000;
  base.max_pulse_us = 2000;
  base.min_position = -1.0;
  base.max_position = 1.0;
  base.max_speed_us_per_second = 200;

  auto shoulder = base;
  shoulder.name = "shoulder";
  shoulder.channel = 1;
  shoulder.direction = -1;
  shoulder.max_speed_us_per_second = 300;
  return {base, shoulder};
}

TEST(PtyIntegration, UsesRealSerialTransport) {
  ssc32_ros2_control::test::Ssc32Emulator emulator({{0U, 1500U}, {1U, 1600U}});
  auto serial = std::make_unique<ssc32::SerialTransport>(emulator.device(), 115200U);
  ssc32_ros2_control::Driver driver(std::move(serial), calibrations(),
                                    std::chrono::milliseconds(500));

  EXPECT_EQ(driver.version(), "SSC32-V2.03XE");
  const auto positions = driver.synchronize();
  ASSERT_EQ(positions.size(), 2U);
  EXPECT_DOUBLE_EQ(positions[0], 0.0);
  EXPECT_DOUBLE_EQ(positions[1], -0.2);
  EXPECT_TRUE(driver.write_positions({0.1, -0.2}));
  driver.cancel();

  ASSERT_TRUE(emulator.wait_for_move_count(1U, std::chrono::seconds(1)));
  ASSERT_TRUE(emulator.wait_for_command_count(4U, std::chrono::seconds(1)));
  EXPECT_EQ(emulator.pulse(0U), 1550U);
  EXPECT_EQ(emulator.pulse(1U), 1600U);
  EXPECT_EQ(emulator.commands().back(), std::string("\x1B\r", 2U));
  EXPECT_TRUE(emulator.error().empty()) << emulator.error();
}

}  // namespace
