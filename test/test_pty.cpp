// SPDX-License-Identifier: BSD-3-Clause
#include <fcntl.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "ssc32/serial_transport.hpp"
#include "ssc32_ros2_control/driver.hpp"

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
        const std::string pulses{static_cast<char>(150), static_cast<char>(160)};
        write_all(master, pulses);
      } else if (command == "#0 P1550 S200 #1 P1600 S300 \r" || command == "ESC\r") {
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
  int master = -1;
  std::thread worker;
  std::string simulator_error;
  try {
    std::string slave_path;
    master = create_pty(slave_path);
    worker = std::thread(simulator, master, std::ref(simulator_error));
    {
      auto serial = std::make_unique<ssc32::SerialTransport>(slave_path, 115200U);
      ssc32_ros2_control::Driver driver(std::move(serial), calibrations(),
                                        std::chrono::milliseconds(500));
      EXPECT_EQ(driver.version(), "SSC32-V2.03XE");
      const auto positions = driver.synchronize();
      ASSERT_EQ(positions.size(), 2U);
      EXPECT_DOUBLE_EQ(positions[0], 0.0);
      EXPECT_DOUBLE_EQ(positions[1], -0.2);
      driver.write_positions({0.1, -0.2});
      driver.cancel();
    }
    close(master);
    master = -1;
    worker.join();
    ASSERT_TRUE(simulator_error.empty()) << simulator_error;
  } catch (...) {
    if (master >= 0) {
      close(master);
    }
    if (worker.joinable()) {
      worker.join();
    }
    throw;
  }
}

}  // namespace
