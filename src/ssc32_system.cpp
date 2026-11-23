// SPDX-License-Identifier: BSD-3-Clause
#include "ssc32_ros2_control/ssc32_system.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "ssc32/serial_transport.hpp"

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

std::string interface_name(const JointCalibration& calibration) {
  return calibration.name + "/" + hardware_interface::HW_IF_POSITION;
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

hardware_interface::CallbackReturn Ssc32System::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    const auto& info = get_hardware_info();
    device_ = require_parameter(info.hardware_parameters, "device", "hardware");
    baud_rate_ = checked_integer<unsigned int>(
        parse_long(require_parameter(info.hardware_parameters, "baud_rate", "hardware"),
                   "hardware baud_rate"),
        1L, static_cast<long>(std::numeric_limits<unsigned int>::max()), "hardware baud_rate");
    timeout_ = std::chrono::milliseconds(checked_integer<int>(
        parse_long(require_parameter(info.hardware_parameters, "timeout_ms", "hardware"),
                   "hardware timeout_ms"),
        1L, 60000L, "hardware timeout_ms"));

    if (info.joints.empty()) {
      throw std::invalid_argument("hardware must define at least one joint");
    }
    calibrations_.clear();
    calibrations_.reserve(info.joints.size());
    std::set<std::uint8_t> channels;
    for (const auto& joint : info.joints) {
      auto calibration = parse_joint(joint);
      if (!channels.insert(calibration.channel).second) {
        throw std::invalid_argument("duplicate SSC-32 channel " +
                                    std::to_string(calibration.channel));
      }
      calibrations_.push_back(std::move(calibration));
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Invalid SSC-32 hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_configure(const rclcpp_lifecycle::State&) {
  try {
    auto serial = std::make_unique<ssc32::SerialTransport>(device_, baud_rate_);
    driver_ = std::make_unique<Driver>(std::move(serial), calibrations_, timeout_);
    const auto firmware = driver_->version();
    if (firmware.empty()) {
      throw std::runtime_error("controller returned an empty firmware version");
    }
    if (!synchronize_interfaces()) {
      throw std::runtime_error("failed to synchronize joint interfaces");
    }
    RCLCPP_INFO(get_logger(), "Connected to SSC-32 controller: %s", firmware.c_str());
  } catch (const std::exception& error) {
    driver_.reset();
    RCLCPP_ERROR(get_logger(), "Failed to configure SSC-32: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_cleanup(const rclcpp_lifecycle::State&) {
  cancel_and_reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_activate(const rclcpp_lifecycle::State&) {
  if (!driver_ || !synchronize_interfaces()) {
    RCLCPP_ERROR(get_logger(), "Cannot activate an unsynchronized SSC-32 controller");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_deactivate(const rclcpp_lifecycle::State&) {
  if (driver_) {
    try {
      driver_->cancel();
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "SSC-32 cancel failed during deactivation: %s", error.what());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_shutdown(const rclcpp_lifecycle::State&) {
  cancel_and_reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_error(const rclcpp_lifecycle::State&) {
  cancel_and_reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ssc32System::read(const rclcpp::Time&, const rclcpp::Duration&) {
  try {
    if (!driver_) {
      throw std::runtime_error("driver is not configured");
    }
    const auto positions = driver_->read_positions();
    for (std::size_t index = 0; index < positions.size(); ++index) {
      set_state(interface_name(calibrations_[index]), positions[index]);
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "SSC-32 read failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ssc32System::write(const rclcpp::Time&, const rclcpp::Duration&) {
  try {
    if (!driver_) {
      throw std::runtime_error("driver is not configured");
    }
    std::vector<double> positions;
    positions.reserve(calibrations_.size());
    for (const auto& calibration : calibrations_) {
      positions.push_back(get_command<double>(interface_name(calibration)));
    }
    driver_->write_positions(positions);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "SSC-32 write failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool Ssc32System::synchronize_interfaces() {
  try {
    const auto positions = driver_->synchronize();
    for (std::size_t index = 0; index < positions.size(); ++index) {
      const auto name = interface_name(calibrations_[index]);
      set_state(name, positions[index]);
      set_command(name, positions[index]);
    }
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "SSC-32 synchronization failed: %s", error.what());
    return false;
  }
}

void Ssc32System::cancel_and_reset() noexcept {
  if (driver_) {
    try {
      driver_->cancel();
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "SSC-32 cancel failed during cleanup: %s", error.what());
    }
  }
  driver_.reset();
}

}  // namespace ssc32_ros2_control

PLUGINLIB_EXPORT_CLASS(ssc32_ros2_control::Ssc32System, hardware_interface::SystemInterface)
