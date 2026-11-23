// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <string>

namespace ssc32_ros2_control {

struct JointCalibration {
  std::string name;
  std::uint8_t channel{};
  std::uint16_t zero_pulse_us{1500};
  double pulse_us_per_unit{};
  int direction{1};
  std::uint16_t min_pulse_us{500};
  std::uint16_t max_pulse_us{2500};
  double min_position{};
  double max_position{};
  std::uint16_t max_speed_us_per_second{};

  void validate() const;
  [[nodiscard]] std::uint16_t position_to_pulse(double position) const;
  [[nodiscard]] double pulse_to_position(std::uint16_t pulse_us) const;
};

}  // namespace ssc32_ros2_control
