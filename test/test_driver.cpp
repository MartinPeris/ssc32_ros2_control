// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ssc32/transport.hpp"
#include "ssc32_ros2_control/driver.hpp"

namespace {

class FakeTransport final : public ssc32::Transport {
 public:
  void write_all(const std::string_view data) override { writes.emplace_back(data); }

  std::vector<std::uint8_t> read_exact(std::size_t size, std::chrono::milliseconds) override {
    if (exact_responses.empty() || (enforce_exact_size && exact_responses.front().size() != size)) {
      throw std::runtime_error("unexpected exact read");
    }
    auto response = std::move(exact_responses.front());
    exact_responses.pop_front();
    return response;
  }

  std::string read_until(char, std::size_t, std::chrono::milliseconds) override {
    return "SSC32-V2.03XE";
  }

  std::vector<std::string> writes;
  std::deque<std::vector<std::uint8_t>> exact_responses;
  bool enforce_exact_size{true};
};

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

TEST(Driver, IdentifiesSynchronizesAndBatchesCommands) {
  auto transport = std::make_unique<FakeTransport>();
  auto* fake = transport.get();
  fake->exact_responses.push_back({150U, 160U});
  ssc32_ros2_control::Driver driver(std::move(transport), calibrations(),
                                    std::chrono::milliseconds(100));

  EXPECT_EQ(driver.version(), "SSC32-V2.03XE");
  const auto positions = driver.synchronize();
  ASSERT_EQ(positions.size(), 2U);
  EXPECT_DOUBLE_EQ(positions[0], 0.0);
  EXPECT_DOUBLE_EQ(positions[1], -0.2);
  ASSERT_EQ(fake->writes.size(), 2U);
  EXPECT_EQ(fake->writes[0], "VER\r");
  EXPECT_EQ(fake->writes[1], "QP0QP1\r");

  EXPECT_TRUE(driver.write_positions({0.1, -0.2}));
  ASSERT_EQ(fake->writes.size(), 3U);
  EXPECT_EQ(fake->writes[2], "#0 P1550 S200 #1 P1600 S300 \r");

  EXPECT_FALSE(driver.write_positions({0.1, -0.2}));
  EXPECT_EQ(fake->writes.size(), 3U);
}

TEST(Driver, ValidatesPositionCountBeforeWriting) {
  auto transport = std::make_unique<FakeTransport>();
  ssc32_ros2_control::Driver driver(std::move(transport), calibrations(),
                                    std::chrono::milliseconds(100));
  EXPECT_THROW(static_cast<void>(driver.write_positions({0.0})), std::invalid_argument);
}

TEST(Driver, RejectsInvalidConstructionAndPulseCounts) {
  EXPECT_THROW(static_cast<void>(ssc32_ros2_control::Driver(nullptr, calibrations(),
                                                            std::chrono::milliseconds(100))),
               std::invalid_argument);
  EXPECT_THROW(static_cast<void>(ssc32_ros2_control::Driver(std::make_unique<FakeTransport>(), {},
                                                            std::chrono::milliseconds(100))),
               std::invalid_argument);
  EXPECT_THROW(
      static_cast<void>(ssc32_ros2_control::Driver(std::make_unique<FakeTransport>(),
                                                   calibrations(), std::chrono::milliseconds(0))),
      std::invalid_argument);

  auto transport = std::make_unique<FakeTransport>();
  auto* fake = transport.get();
  fake->enforce_exact_size = false;
  fake->exact_responses.push_back({150U});
  ssc32_ros2_control::Driver driver(std::move(transport), calibrations(),
                                    std::chrono::milliseconds(100));
  EXPECT_THROW(static_cast<void>(driver.read_positions()), std::runtime_error);
}

TEST(Driver, OmitsOptionalSpeedAndCancels) {
  auto values = calibrations();
  values.front().max_speed_us_per_second = 0U;
  auto transport = std::make_unique<FakeTransport>();
  auto* fake = transport.get();
  ssc32_ros2_control::Driver driver(std::move(transport), std::move(values),
                                    std::chrono::milliseconds(100));
  EXPECT_TRUE(driver.write_positions({0.1, -0.2}));
  ASSERT_EQ(fake->writes.size(), 1U);
  EXPECT_EQ(fake->writes.front(), "#0 P1550 #1 P1600 S300 \r");
  driver.cancel();
  EXPECT_EQ(fake->writes.back(), std::string("\x1B\r", 2U));
}

}  // namespace
