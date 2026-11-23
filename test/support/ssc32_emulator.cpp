// SPDX-License-Identifier: BSD-3-Clause
#include "support/ssc32_emulator.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <exception>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace ssc32_ros2_control::test {
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
      throw std::runtime_error("emulator write failed");
    }
    offset += static_cast<std::size_t>(count);
  }
}

unsigned int parse_unsigned(const std::string_view text, const std::string& context) {
  unsigned int value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    throw std::runtime_error("invalid " + context + " in SSC-32 command");
  }
  return value;
}

}  // namespace

Ssc32Emulator::Ssc32Emulator(std::map<std::uint8_t, std::uint16_t> initial_pulses)
    : pulses_(std::move(initial_pulses)) {
  master_ = create_pty(device_);
  worker_ = std::thread(&Ssc32Emulator::run, this);
}

Ssc32Emulator::~Ssc32Emulator() {
  const int descriptor = master_.exchange(-1);
  if (descriptor >= 0) {
    close(descriptor);
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

const std::string& Ssc32Emulator::device() const noexcept { return device_; }

std::vector<std::string> Ssc32Emulator::commands() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return commands_;
}

std::vector<std::string> Ssc32Emulator::move_commands() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return move_commands_;
}

std::uint16_t Ssc32Emulator::pulse(const std::uint8_t channel) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = pulses_.find(channel);
  return found == pulses_.end() ? 1500U : found->second;
}

std::string Ssc32Emulator::error() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}

bool Ssc32Emulator::wait_for_move_count(const std::size_t count,
                                        const std::chrono::milliseconds timeout) const {
  std::unique_lock<std::mutex> lock(mutex_);
  return command_condition_.wait_for(lock, timeout,
                                     [this, count] { return move_commands_.size() >= count; });
}

bool Ssc32Emulator::wait_for_command_count(const std::size_t count,
                                           const std::chrono::milliseconds timeout) const {
  std::unique_lock<std::mutex> lock(mutex_);
  return command_condition_.wait_for(lock, timeout,
                                     [this, count] { return commands_.size() >= count; });
}

void Ssc32Emulator::set_response_delay(const std::chrono::milliseconds delay) {
  if (delay.count() < 0) {
    throw std::invalid_argument("response delay cannot be negative");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  response_delay_ = delay;
}

void Ssc32Emulator::fault_next_response(const ResponseFault fault) {
  std::lock_guard<std::mutex> lock(mutex_);
  next_fault_ = fault;
}

void Ssc32Emulator::run() noexcept {
  try {
    std::string command;
    char character = '\0';
    int descriptor = master_.load();
    while (descriptor >= 0 && read(descriptor, &character, 1U) == 1) {
      command += character;
      if (character == '\r') {
        handle_command(command);
        command.clear();
      }
      descriptor = master_.load();
    }
  } catch (const std::exception& exception) {
    record_error(exception.what());
  } catch (...) {
    record_error("emulator failed with an unknown error");
  }
}

void Ssc32Emulator::handle_command(const std::string& command) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.push_back(command);
  }
  command_condition_.notify_all();

  if (command == "VER\r") {
    respond("SSC32-V2.03XE\r");
    return;
  }
  if (command == std::string{"\x1B\r", 2U}) {
    return;
  }
  if (command.rfind("QP", 0U) == 0U) {
    std::string response;
    std::size_t position = 0U;
    while (position + 2U <= command.size() && command.compare(position, 2U, "QP") == 0) {
      position += 2U;
      const auto end = command.find_first_not_of("0123456789", position);
      if (end == position || end == std::string::npos) {
        throw std::runtime_error("malformed pulse query: " + command);
      }
      const auto channel =
          parse_unsigned(std::string_view(command).substr(position, end - position), "channel");
      if (channel > 31U) {
        throw std::runtime_error("pulse query channel is outside 0..31");
      }
      const auto pulse_value = pulse(static_cast<std::uint8_t>(channel));
      response.push_back(static_cast<char>(std::clamp<unsigned int>(pulse_value / 10U, 0U, 255U)));
      position = end;
    }
    if (position + 1U != command.size() || command[position] != '\r') {
      throw std::runtime_error("malformed pulse query: " + command);
    }
    respond(response);
    return;
  }
  if (!command.empty() && command.front() == '#') {
    std::size_t position = 0U;
    std::map<std::uint8_t, std::uint16_t> updates;
    while (position < command.size() && command[position] == '#') {
      const auto channel_end = command.find(' ', position);
      if (channel_end == std::string::npos) {
        throw std::runtime_error("malformed move command: " + command);
      }
      const auto channel = parse_unsigned(
          std::string_view(command).substr(position + 1U, channel_end - position - 1U), "channel");
      const auto pulse_start = command.find('P', channel_end);
      if (pulse_start == std::string::npos) {
        throw std::runtime_error("move command has no pulse: " + command);
      }
      const auto pulse_end = command.find_first_not_of("0123456789", pulse_start + 1U);
      if (pulse_end == pulse_start + 1U || pulse_end == std::string::npos) {
        throw std::runtime_error("malformed pulse in move command: " + command);
      }
      const auto pulse_value = parse_unsigned(
          std::string_view(command).substr(pulse_start + 1U, pulse_end - pulse_start - 1U),
          "pulse");
      if (channel > 31U || pulse_value > 65535U) {
        throw std::runtime_error("move command value is outside the supported range");
      }
      updates[static_cast<std::uint8_t>(channel)] = static_cast<std::uint16_t>(pulse_value);
      const auto next = command.find('#', pulse_end);
      if (next == std::string::npos) {
        break;
      }
      position = next;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& update : updates) {
        pulses_[update.first] = update.second;
      }
      move_commands_.push_back(command);
    }
    command_condition_.notify_all();
    return;
  }
  throw std::runtime_error("unexpected SSC-32 command: " + command);
}

void Ssc32Emulator::respond(const std::string& response) {
  std::chrono::milliseconds delay{0};
  ResponseFault fault = ResponseFault::kNone;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    delay = response_delay_;
    fault = next_fault_;
    next_fault_ = ResponseFault::kNone;
  }
  if (delay.count() > 0) {
    std::this_thread::sleep_for(delay);
  }
  if (fault == ResponseFault::kDrop) {
    return;
  }
  if (fault == ResponseFault::kDisconnect) {
    const int descriptor = master_.exchange(-1);
    if (descriptor >= 0) {
      close(descriptor);
    }
    return;
  }
  if (fault == ResponseFault::kTruncate) {
    if (!response.empty()) {
      write_all(master_.load(), std::string_view(response).substr(0U, response.size() - 1U));
    }
    return;
  }
  write_all(master_.load(), response);
}

void Ssc32Emulator::record_error(const std::string& error) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  error_ = error;
  command_condition_.notify_all();
}

}  // namespace ssc32_ros2_control::test
