// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ssc32/controller.hpp"
#include "ssc32/transport.hpp"
#include "ssc32_ros2_control/joint_calibration.hpp"

namespace ssc32_ros2_control {

class Driver final {
 public:
  Driver(std::unique_ptr<ssc32::Transport> transport, std::vector<JointCalibration> calibrations,
         std::chrono::milliseconds timeout);

  [[nodiscard]] std::string version();
  [[nodiscard]] std::vector<double> synchronize();
  [[nodiscard]] std::vector<double> read_positions();
  [[nodiscard]] bool write_positions(const std::vector<double>& positions);
  void cancel();

 private:
  std::unique_ptr<ssc32::Transport> transport_;
  ssc32::Controller controller_;
  std::vector<JointCalibration> calibrations_;
  std::vector<std::uint16_t> last_commanded_pulses_;
  std::chrono::milliseconds timeout_;
};

}  // namespace ssc32_ros2_control
