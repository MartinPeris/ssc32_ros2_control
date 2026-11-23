// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ssc32_ros2_control::test {

class Ssc32Emulator final {
 public:
  enum class ResponseFault { kNone, kDrop, kTruncate, kDisconnect };

  explicit Ssc32Emulator(std::map<std::uint8_t, std::uint16_t> initial_pulses = {});
  ~Ssc32Emulator();

  Ssc32Emulator(const Ssc32Emulator&) = delete;
  Ssc32Emulator& operator=(const Ssc32Emulator&) = delete;

  [[nodiscard]] const std::string& device() const noexcept;
  [[nodiscard]] std::vector<std::string> commands() const;
  [[nodiscard]] std::vector<std::string> move_commands() const;
  [[nodiscard]] std::uint16_t pulse(std::uint8_t channel) const;
  [[nodiscard]] std::string error() const;
  [[nodiscard]] bool wait_for_move_count(std::size_t count,
                                         std::chrono::milliseconds timeout) const;

  void set_response_delay(std::chrono::milliseconds delay);
  void fault_next_response(ResponseFault fault);

 private:
  void run() noexcept;
  void handle_command(const std::string& command);
  void respond(const std::string& response);
  void record_error(const std::string& error) noexcept;

  std::atomic<int> master_{-1};
  std::string device_;
  mutable std::mutex mutex_;
  mutable std::condition_variable command_condition_;
  std::map<std::uint8_t, std::uint16_t> pulses_;
  std::vector<std::string> commands_;
  std::vector<std::string> move_commands_;
  std::chrono::milliseconds response_delay_{0};
  ResponseFault next_fault_{ResponseFault::kNone};
  std::string error_;
  std::thread worker_;
};

}  // namespace ssc32_ros2_control::test
