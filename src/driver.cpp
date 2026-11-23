// SPDX-License-Identifier: BSD-3-Clause
#include "ssc32_ros2_control/driver.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>

namespace ssc32_ros2_control {
namespace {

ssc32::Transport& require_transport(const std::unique_ptr<ssc32::Transport>& transport) {
  if (!transport) {
    throw std::invalid_argument("transport must not be null");
  }
  return *transport;
}

}  // namespace

Driver::Driver(std::unique_ptr<ssc32::Transport> transport,
               std::vector<JointCalibration> calibrations, const std::chrono::milliseconds timeout)
    : transport_(std::move(transport)),
      controller_(require_transport(transport_)),
      calibrations_(std::move(calibrations)),
      timeout_(timeout) {
  if (calibrations_.empty()) {
    throw std::invalid_argument("at least one joint calibration is required");
  }
  if (timeout_.count() <= 0) {
    throw std::invalid_argument("timeout must be positive");
  }
  for (const auto& calibration : calibrations_) {
    calibration.validate();
  }
}

std::string Driver::version() { return controller_.version(timeout_); }

std::vector<double> Driver::synchronize() {
  const auto positions = read_positions();
  last_commanded_pulses_.clear();
  last_commanded_pulses_.reserve(positions.size());
  for (std::size_t index = 0; index < positions.size(); ++index) {
    last_commanded_pulses_.push_back(calibrations_[index].position_to_pulse(positions[index]));
  }
  return positions;
}

std::vector<double> Driver::read_positions() {
  std::vector<std::uint8_t> channels;
  channels.reserve(calibrations_.size());
  for (const auto& calibration : calibrations_) {
    channels.push_back(calibration.channel);
  }
  const auto pulses = controller_.query_pulse_widths(channels, timeout_);
  if (pulses.size() != calibrations_.size()) {
    throw std::runtime_error("SSC-32 returned an unexpected pulse count");
  }

  std::vector<double> positions;
  positions.reserve(pulses.size());
  for (std::size_t index = 0; index < pulses.size(); ++index) {
    positions.push_back(calibrations_[index].pulse_to_position(pulses[index]));
  }
  return positions;
}

bool Driver::write_positions(const std::vector<double>& positions) {
  if (positions.size() != calibrations_.size()) {
    throw std::invalid_argument("position count does not match configured joints");
  }

  std::vector<std::uint16_t> pulses;
  pulses.reserve(positions.size());
  for (std::size_t index = 0; index < positions.size(); ++index) {
    pulses.push_back(calibrations_[index].position_to_pulse(positions[index]));
  }
  if (pulses == last_commanded_pulses_) {
    return false;
  }

  std::vector<ssc32::ServoCommand> commands;
  commands.reserve(pulses.size());
  for (std::size_t index = 0; index < pulses.size(); ++index) {
    ssc32::ServoCommand command;
    command.channel = calibrations_[index].channel;
    command.pulse_width_us = pulses[index];
    if (calibrations_[index].max_speed_us_per_second > 0U) {
      command.speed_us_per_second = calibrations_[index].max_speed_us_per_second;
    }
    commands.push_back(command);
  }
  controller_.move(commands);
  last_commanded_pulses_ = std::move(pulses);
  return true;
}

void Driver::cancel() { controller_.cancel(); }

}  // namespace ssc32_ros2_control
