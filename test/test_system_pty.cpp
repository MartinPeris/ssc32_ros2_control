// SPDX-License-Identifier: BSD-3-Clause
#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <exception>
#include <fstream>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ssc32_ros2_control/ssc32_system.hpp"

namespace {

int create_pty(std::string& slave_path) {
  const int master = posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) {
    throw std::runtime_error("failed to create pseudoterminal");
  }
  const char* path = ptsname(master);
  if (path == nullptr) {
    close(master);
    throw std::runtime_error("failed to resolve pseudoterminal path");
  }
  slave_path = path;
  return master;
}

void write_all(const int descriptor, const std::string_view value) {
  std::size_t offset = 0;
  while (offset < value.size()) {
    const auto count = write(descriptor, value.data() + offset, value.size() - offset);
    if (count <= 0) {
      throw std::runtime_error("simulator write failed");
    }
    offset += static_cast<std::size_t>(count);
  }
}

void simulator(const int master, std::string& error) noexcept {
  try {
    std::string command;
    char character = '\0';
    while (read(master, &character, 1U) == 1) {
      command += character;
      if (character != '\r') {
        continue;
      }
      if (command == "VER\r") {
        write_all(master, "SSC32-V2.03XE\r");
      } else if (command == "QP0QP1\r") {
        const std::string pulses{static_cast<char>(150), static_cast<char>(150)};
        write_all(master, pulses);
      } else if (command == "#0 P1550 S100 #1 P1500 S75 \r" || command == "ESC\r") {
        // Write-only commands have no response.
      } else {
        throw std::runtime_error("unexpected simulator command: " + command);
      }
      command.clear();
    }
  } catch (const std::exception& exception) {
    error = exception.what();
  } catch (...) {
    error = "simulator failed with an unknown error";
  }
}

std::string example_urdf(const std::string& device) {
  const std::string path = std::string(SSC32_ROS2_CONTROL_SOURCE_DIR) + "/urdf/example.urdf";
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open example URDF");
  }
  std::string urdf((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const std::string placeholder = "/dev/ttyUSB0";
  const auto position = urdf.find(placeholder);
  if (position == std::string::npos) {
    throw std::runtime_error("example URDF has no device placeholder");
  }
  urdf.replace(position, placeholder.size(), device);
  return urdf;
}

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

TEST(SystemPtyIntegration, ExercisesLifecycleAndInterfaces) {
  int master = -1;
  std::thread worker;
  std::string simulator_error;
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  try {
    std::string slave_path;
    master = create_pty(slave_path);
    worker = std::thread(simulator, master, std::ref(simulator_error));

    const auto resources =
        hardware_interface::parse_control_resources_from_urdf(example_urdf(slave_path));
    require(resources.size() == 1U, "unexpected hardware resource count");
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = resources.front();

    ssc32_ros2_control::Ssc32System system;
    require(system.on_init(params) == hardware_interface::CallbackReturn::SUCCESS,
            "system initialization failed");
    const auto states = system.on_export_state_interfaces();
    const auto commands = system.on_export_command_interfaces();
    require(states.size() == 2U, "unexpected state interface count");
    require(commands.size() == 2U, "unexpected command interface count");

    const rclcpp_lifecycle::State state;
    require(system.on_configure(state) == hardware_interface::CallbackReturn::SUCCESS,
            "system configuration failed");
    require(system.on_activate(state) == hardware_interface::CallbackReturn::SUCCESS,
            "system activation failed");
    require(system.read(rclcpp::Time(0), rclcpp::Duration(0, 20000000)) ==
                hardware_interface::return_type::OK,
            "system read failed");
    require(commands.front()->set_value(0.1), "failed to set command interface");
    require(system.write(rclcpp::Time(0), rclcpp::Duration(0, 20000000)) ==
                hardware_interface::return_type::OK,
            "system write failed");
    require(system.on_deactivate(state) == hardware_interface::CallbackReturn::SUCCESS,
            "system deactivation failed");
    require(system.on_cleanup(state) == hardware_interface::CallbackReturn::SUCCESS,
            "system cleanup failed");

    close(master);
    master = -1;
    worker.join();
    require(simulator_error.empty(), simulator_error);
  } catch (...) {
    if (master >= 0) {
      close(master);
    }
    if (worker.joinable()) {
      worker.join();
    }
    rclcpp::shutdown();
    throw;
  }
  rclcpp::shutdown();
}

}  // namespace
