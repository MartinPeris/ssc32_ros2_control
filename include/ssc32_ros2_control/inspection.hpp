// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "ssc32_ros2_control/configuration.hpp"

namespace ssc32_ros2_control {

[[nodiscard]] HardwareConfiguration inspect_hardware_resources(
    const std::vector<hardware_interface::HardwareInfo>& resources,
    const std::optional<std::string>& requested_name = std::nullopt);
[[nodiscard]] std::string render_configuration_text(const HardwareConfiguration& configuration);
[[nodiscard]] std::string render_configuration_json(const HardwareConfiguration& configuration);

}  // namespace ssc32_ros2_control
