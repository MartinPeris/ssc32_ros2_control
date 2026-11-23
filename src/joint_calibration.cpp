// SPDX-License-Identifier: BSD-3-Clause
#include "ssc32_ros2_control/joint_calibration.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "ssc32/protocol.hpp"

namespace ssc32_ros2_control {

void JointCalibration::validate() const {
  if (name.empty()) {
    throw std::invalid_argument("joint name must not be empty");
  }
  if (channel > ssc32::kMaxChannel) {
    throw std::invalid_argument(name + ": channel must be in [0, 31]");
  }
  if (!std::isfinite(pulse_us_per_unit) || pulse_us_per_unit <= 0.0) {
    throw std::invalid_argument(name + ": pulse_us_per_unit must be finite and positive");
  }
  if (direction != -1 && direction != 1) {
    throw std::invalid_argument(name + ": direction must be -1 or 1");
  }
  if (min_pulse_us < ssc32::kMinPulseWidthUs || max_pulse_us > ssc32::kMaxPulseWidthUs ||
      min_pulse_us >= max_pulse_us) {
    throw std::invalid_argument(name + ": invalid pulse envelope");
  }
  if (zero_pulse_us < min_pulse_us || zero_pulse_us > max_pulse_us) {
    throw std::invalid_argument(name + ": zero pulse is outside the pulse envelope");
  }
  if (!std::isfinite(min_position) || !std::isfinite(max_position) ||
      min_position >= max_position) {
    throw std::invalid_argument(name + ": invalid position limits");
  }

  const auto min_endpoint = position_to_pulse(min_position);
  const auto max_endpoint = position_to_pulse(max_position);
  static_cast<void>(min_endpoint);
  static_cast<void>(max_endpoint);
}

std::uint16_t JointCalibration::position_to_pulse(const double position) const {
  if (!std::isfinite(position) || position < min_position || position > max_position) {
    throw std::out_of_range(name + ": position is outside configured limits");
  }
  const double pulse = static_cast<double>(zero_pulse_us) +
                       static_cast<double>(direction) * position * pulse_us_per_unit;
  if (!std::isfinite(pulse) || pulse < static_cast<double>(min_pulse_us) ||
      pulse > static_cast<double>(max_pulse_us)) {
    throw std::out_of_range(name + ": position maps outside the pulse envelope");
  }
  const long rounded = std::lround(pulse);
  if (rounded < 0L || rounded > static_cast<long>(std::numeric_limits<std::uint16_t>::max())) {
    throw std::out_of_range(name + ": pulse conversion overflow");
  }
  return static_cast<std::uint16_t>(rounded);
}

double JointCalibration::pulse_to_position(const std::uint16_t pulse_us) const {
  if (pulse_us < min_pulse_us || pulse_us > max_pulse_us) {
    throw std::out_of_range(name + ": reported pulse is outside the pulse envelope");
  }
  const double position = (static_cast<double>(pulse_us) - static_cast<double>(zero_pulse_us)) /
                          (static_cast<double>(direction) * pulse_us_per_unit);
  if (position < min_position || position > max_position) {
    throw std::out_of_range(name + ": reported pulse maps outside position limits");
  }
  return position;
}

}  // namespace ssc32_ros2_control
