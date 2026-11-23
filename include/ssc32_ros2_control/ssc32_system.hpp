// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ssc32_ros2_control/configuration.hpp"
#include "ssc32_ros2_control/diagnostics.hpp"
#include "ssc32_ros2_control/driver.hpp"

namespace ssc32_ros2_control {

class Ssc32System final : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  [[nodiscard]] bool synchronize_interfaces();
  void cancel_and_reset() noexcept;

  HardwareConfiguration configuration_;
  std::unique_ptr<Driver> driver_;
  std::unique_ptr<Diagnostics> diagnostics_;
};

}  // namespace ssc32_ros2_control
