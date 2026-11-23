// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "ssc32_ros2_control/joint_calibration.hpp"

namespace {

ssc32_ros2_control::JointCalibration calibration() {
  ssc32_ros2_control::JointCalibration value;
  value.name = "shoulder";
  value.channel = 1;
  value.zero_pulse_us = 1500;
  value.pulse_us_per_unit = 500.0;
  value.direction = -1;
  value.min_pulse_us = 1000;
  value.max_pulse_us = 2000;
  value.min_position = -1.0;
  value.max_position = 1.0;
  value.max_speed_us_per_second = 250;
  return value;
}

TEST(JointCalibration, ConvertsBothDirections) {
  const auto value = calibration();
  EXPECT_EQ(value.position_to_pulse(0.5), 1250U);
  EXPECT_DOUBLE_EQ(value.pulse_to_position(1750U), -0.5);
}

TEST(JointCalibration, RejectsPositionOutsideEnvelope) {
  const auto value = calibration();
  EXPECT_THROW(static_cast<void>(value.position_to_pulse(1.01)), std::out_of_range);
}

TEST(JointCalibration, RejectsInvalidDirection) {
  auto value = calibration();
  value.direction = 0;
  EXPECT_THROW(value.validate(), std::invalid_argument);
}

TEST(JointCalibration, RejectsPositionRangeThatExceedsPulseEnvelope) {
  auto value = calibration();
  value.max_position = 1.1;
  EXPECT_THROW(value.validate(), std::out_of_range);
}

TEST(JointCalibration, RejectsReportedPulseOutsidePositionLimits) {
  auto value = calibration();
  value.min_position = -0.5;
  value.max_position = 0.5;
  value.validate();
  EXPECT_THROW(static_cast<void>(value.pulse_to_position(1000U)), std::out_of_range);
}

TEST(JointCalibration, RejectsEveryInvalidCalibrationField) {
  auto value = calibration();
  value.name.clear();
  EXPECT_THROW(value.validate(), std::invalid_argument);
  value = calibration();
  value.channel = 32U;
  EXPECT_THROW(value.validate(), std::invalid_argument);
  value = calibration();
  value.pulse_us_per_unit = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(value.validate(), std::invalid_argument);
  value = calibration();
  value.min_pulse_us = 499U;
  EXPECT_THROW(value.validate(), std::invalid_argument);
  value = calibration();
  value.zero_pulse_us = 999U;
  EXPECT_THROW(value.validate(), std::invalid_argument);
  value = calibration();
  value.min_position = value.max_position;
  EXPECT_THROW(value.validate(), std::invalid_argument);
}

TEST(JointCalibration, RejectsNonFinitePosition) {
  const auto value = calibration();
  EXPECT_THROW(static_cast<void>(value.position_to_pulse(std::numeric_limits<double>::infinity())),
               std::out_of_range);
}

}  // namespace
