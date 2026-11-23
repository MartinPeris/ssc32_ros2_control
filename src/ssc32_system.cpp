// SPDX-License-Identifier: BSD-3-Clause
#include "ssc32_ros2_control/ssc32_system.hpp"

#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "ssc32/serial_transport.hpp"

namespace ssc32_ros2_control {
namespace {

std::string interface_name(const JointCalibration& calibration) {
  return calibration.name + "/" + hardware_interface::HW_IF_POSITION;
}

}  // namespace

hardware_interface::CallbackReturn Ssc32System::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    configuration_ = parse_hardware_configuration(get_hardware_info());
    diagnostics_ = std::make_unique<Diagnostics>(get_node(), configuration_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Invalid SSC-32 hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_configure(const rclcpp_lifecycle::State&) {
  try {
    auto serial =
        std::make_unique<ssc32::SerialTransport>(configuration_.device, configuration_.baud_rate);
    driver_ =
        std::make_unique<Driver>(std::move(serial), configuration_.joints, configuration_.timeout);
    const auto firmware = driver_->version();
    if (firmware.empty()) {
      throw std::runtime_error("controller returned an empty firmware version");
    }
    if (!synchronize_interfaces()) {
      driver_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }
    diagnostics_->mark_configured(firmware);
    RCLCPP_INFO(get_logger(), "Connected to SSC-32 controller: %s", firmware.c_str());
  } catch (const std::exception& error) {
    driver_.reset();
    diagnostics_->record_error("connect", error.what());
    RCLCPP_ERROR(get_logger(), "Failed to configure SSC-32: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_cleanup(const rclcpp_lifecycle::State&) {
  cancel_and_reset();
  diagnostics_->mark_unconfigured();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_activate(const rclcpp_lifecycle::State&) {
  if (!driver_) {
    diagnostics_->record_error("activate", "driver is not configured");
    RCLCPP_ERROR(get_logger(), "Cannot activate an unconfigured SSC-32 controller");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!synchronize_interfaces()) {
    RCLCPP_ERROR(get_logger(), "Cannot activate an unsynchronized SSC-32 controller");
    return hardware_interface::CallbackReturn::ERROR;
  }
  diagnostics_->mark_active();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_deactivate(const rclcpp_lifecycle::State&) {
  if (driver_) {
    try {
      driver_->cancel();
      diagnostics_->mark_inactive();
    } catch (const std::exception& error) {
      diagnostics_->record_error("cancel", error.what());
      RCLCPP_WARN(get_logger(), "SSC-32 cancel failed during deactivation: %s", error.what());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_shutdown(const rclcpp_lifecycle::State&) {
  cancel_and_reset();
  diagnostics_->mark_unconfigured();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ssc32System::on_error(const rclcpp_lifecycle::State&) {
  diagnostics_->ensure_error("lifecycle", "Hardware lifecycle entered the error state");
  cancel_and_reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ssc32System::read(const rclcpp::Time&, const rclcpp::Duration&) {
  const auto started = std::chrono::steady_clock::now();
  try {
    if (!driver_) {
      throw std::runtime_error("driver is not configured");
    }
    const auto positions = driver_->read_positions();
    for (std::size_t index = 0; index < positions.size(); ++index) {
      set_state(interface_name(configuration_.joints[index]), positions[index]);
    }
    diagnostics_->record_read_success(std::chrono::steady_clock::now() - started);
  } catch (const std::exception& error) {
    diagnostics_->record_error("read", error.what());
    RCLCPP_ERROR(get_logger(), "SSC-32 read failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ssc32System::write(const rclcpp::Time&, const rclcpp::Duration&) {
  const auto started = std::chrono::steady_clock::now();
  try {
    if (!driver_) {
      throw std::runtime_error("driver is not configured");
    }
    std::vector<double> positions;
    positions.reserve(configuration_.joints.size());
    for (const auto& calibration : configuration_.joints) {
      positions.push_back(get_command<double>(interface_name(calibration)));
    }
    const bool transmitted = driver_->write_positions(positions);
    diagnostics_->record_write_success(std::chrono::steady_clock::now() - started, transmitted);
  } catch (const std::exception& error) {
    diagnostics_->record_error("write", error.what());
    RCLCPP_ERROR(get_logger(), "SSC-32 write failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool Ssc32System::synchronize_interfaces() {
  const auto started = std::chrono::steady_clock::now();
  try {
    const auto positions = driver_->synchronize();
    for (std::size_t index = 0; index < positions.size(); ++index) {
      const auto name = interface_name(configuration_.joints[index]);
      set_state(name, positions[index]);
      set_command(name, positions[index]);
    }
    diagnostics_->record_read_success(std::chrono::steady_clock::now() - started);
    return true;
  } catch (const std::exception& error) {
    diagnostics_->record_error("read", error.what());
    RCLCPP_ERROR(get_logger(), "SSC-32 synchronization failed: %s", error.what());
    return false;
  }
}

void Ssc32System::cancel_and_reset() noexcept {
  if (driver_) {
    try {
      driver_->cancel();
    } catch (const std::exception& error) {
      diagnostics_->record_error("cancel", error.what());
      RCLCPP_WARN(get_logger(), "SSC-32 cancel failed during cleanup: %s", error.what());
    }
  }
  driver_.reset();
}

}  // namespace ssc32_ros2_control

PLUGINLIB_EXPORT_CLASS(ssc32_ros2_control::Ssc32System, hardware_interface::SystemInterface)
