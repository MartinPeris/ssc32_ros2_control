// SPDX-License-Identifier: BSD-3-Clause
#include "ssc32_ros2_control/configuration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ssc32_ros2_control {
namespace {

const std::string& require_parameter(const std::unordered_map<std::string, std::string>& parameters,
                                     const std::string& name, const std::string& context) {
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end() || iterator->second.empty()) {
    throw std::invalid_argument(context + ": missing parameter '" + name + "'");
  }
  return iterator->second;
}

long parse_long(const std::string& value, const std::string& context) {
  std::size_t consumed = 0;
  const long parsed = std::stol(value, &consumed, 10);
  if (consumed != value.size()) {
    throw std::invalid_argument(context + ": expected an integer");
  }
  return parsed;
}

double parse_double(const std::string& value, const std::string& context) {
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(context + ": expected a finite number");
  }
  return parsed;
}

template <typename Integer>
Integer checked_integer(const long value, const long minimum, const long maximum,
                        const std::string& context) {
  if (value < minimum || value > maximum) {
    throw std::out_of_range(context + ": value is outside the supported range");
  }
  return static_cast<Integer>(value);
}

JointCalibration parse_joint(const hardware_interface::ComponentInfo& joint) {
  const std::string context = "joint '" + joint.name + "'";
  if (joint.command_interfaces.size() != 1U ||
      joint.command_interfaces.front().name != hardware_interface::HW_IF_POSITION ||
      joint.command_interfaces.front().data_type != "double") {
    throw std::invalid_argument(context +
                                ": requires exactly one double position command interface");
  }
  if (joint.state_interfaces.size() != 1U ||
      joint.state_interfaces.front().name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces.front().data_type != "double") {
    throw std::invalid_argument(context + ": requires exactly one double position state interface");
  }
  const auto& command = joint.command_interfaces.front();
  if (command.min.empty() || command.max.empty()) {
    throw std::invalid_argument(context + ": position command min and max are required");
  }

  JointCalibration calibration;
  calibration.name = joint.name;
  calibration.channel = checked_integer<std::uint8_t>(
      parse_long(require_parameter(joint.parameters, "channel", context), context + " channel"), 0L,
      31L, context + " channel");
  calibration.zero_pulse_us = checked_integer<std::uint16_t>(
      parse_long(require_parameter(joint.parameters, "zero_pulse_us", context),
                 context + " zero_pulse_us"),
      500L, 2500L, context + " zero_pulse_us");
  calibration.pulse_us_per_unit =
      parse_double(require_parameter(joint.parameters, "pulse_us_per_unit", context),
                   context + " pulse_us_per_unit");
  calibration.direction = checked_integer<int>(
      parse_long(require_parameter(joint.parameters, "direction", context), context + " direction"),
      -1L, 1L, context + " direction");
  calibration.min_pulse_us = checked_integer<std::uint16_t>(
      parse_long(require_parameter(joint.parameters, "min_pulse_us", context),
                 context + " min_pulse_us"),
      500L, 2500L, context + " min_pulse_us");
  calibration.max_pulse_us = checked_integer<std::uint16_t>(
      parse_long(require_parameter(joint.parameters, "max_pulse_us", context),
                 context + " max_pulse_us"),
      500L, 2500L, context + " max_pulse_us");
  calibration.min_position = parse_double(command.min, context + " position min");
  calibration.max_position = parse_double(command.max, context + " position max");

  const auto speed = joint.parameters.find("max_speed_us_per_second");
  if (speed != joint.parameters.end()) {
    calibration.max_speed_us_per_second = checked_integer<std::uint16_t>(
        parse_long(speed->second, context + " max_speed_us_per_second"), 1L, 65535L,
        context + " max_speed_us_per_second");
  }
  calibration.validate();
  return calibration;
}

}  // namespace

HardwareConfiguration parse_hardware_configuration(const hardware_interface::HardwareInfo& info) {
  HardwareConfiguration configuration;
  configuration.name = info.name;
  configuration.device = require_parameter(info.hardware_parameters, "device", "hardware");
  configuration.baud_rate = checked_integer<unsigned int>(
      parse_long(require_parameter(info.hardware_parameters, "baud_rate", "hardware"),
                 "hardware baud_rate"),
      1L, static_cast<long>(std::numeric_limits<unsigned int>::max()), "hardware baud_rate");
  configuration.timeout = std::chrono::milliseconds(checked_integer<int>(
      parse_long(require_parameter(info.hardware_parameters, "timeout_ms", "hardware"),
                 "hardware timeout_ms"),
      1L, 60000L, "hardware timeout_ms"));

  const auto stale_after = info.hardware_parameters.find("diagnostics_stale_after_ms");
  if (stale_after == info.hardware_parameters.end()) {
    configuration.diagnostics_stale_after =
        std::max(std::chrono::milliseconds(1000), configuration.timeout * 4);
  } else {
    configuration.diagnostics_stale_after = std::chrono::milliseconds(
        checked_integer<int>(parse_long(stale_after->second, "hardware diagnostics_stale_after_ms"),
                             1L, 600000L, "hardware diagnostics_stale_after_ms"));
  }

  if (info.joints.empty()) {
    throw std::invalid_argument("hardware must define at least one joint");
  }
  configuration.joints.reserve(info.joints.size());
  std::set<std::uint8_t> channels;
  for (const auto& joint : info.joints) {
    auto calibration = parse_joint(joint);
    if (!channels.insert(calibration.channel).second) {
      throw std::invalid_argument("duplicate SSC-32 channel " +
                                  std::to_string(calibration.channel));
    }
    configuration.joints.push_back(std::move(calibration));
  }
  return configuration;
}

}  // namespace ssc32_ros2_control
