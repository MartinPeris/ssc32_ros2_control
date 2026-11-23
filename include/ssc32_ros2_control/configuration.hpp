// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "ssc32_ros2_control/joint_calibration.hpp"

namespace ssc32_ros2_control {

struct HardwareConfiguration {
  std::string name;
  std::string device;
  unsigned int baud_rate{115200U};
  std::chrono::milliseconds timeout{250};
  std::chrono::milliseconds diagnostics_stale_after{1000};
  std::vector<JointCalibration> joints;
};

[[nodiscard]] HardwareConfiguration parse_hardware_configuration(
    const hardware_interface::HardwareInfo& info);

}  // namespace ssc32_ros2_control
