// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "support/ssc32_emulator.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace {
using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

std::string example_urdf(const std::string& device) {
  const std::string path = std::string(SSC32_ROS2_CONTROL_SOURCE_DIR) + "/urdf/example.urdf";
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open example URDF");
  }
  std::string urdf((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const std::string placeholder = "/dev/ttyUSB0";
  const auto device_position = urdf.find(placeholder);
  if (device_position == std::string::npos) {
    throw std::runtime_error("example URDF has no device placeholder");
  }
  urdf.replace(device_position, placeholder.size(), device);

  const std::string asynchronous = " is_async=\"true\" rw_rate=\"50\"";
  const auto async_position = urdf.find(asynchronous);
  if (async_position == std::string::npos) {
    throw std::runtime_error("example URDF has no asynchronous hardware attributes");
  }
  urdf.erase(async_position, asynchronous.size());
  return urdf;
}

class RclcppScope final {
 public:
  RclcppScope() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  ~RclcppScope() { rclcpp::shutdown(); }
};

class ControlLoop final {
 public:
  explicit ControlLoop(std::shared_ptr<controller_manager::ControllerManager> manager)
      : manager_(std::move(manager)), worker_([this] { run(); }) {}

  ~ControlLoop() {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void run() {
    auto previous = manager_->get_trigger_clock()->now();
    while (running_ && rclcpp::ok()) {
      const auto now = manager_->get_trigger_clock()->now();
      const auto period = now - previous;
      previous = now;
      manager_->read(now, period);
      manager_->update(now, period);
      manager_->write(now, period);
      std::this_thread::sleep_for(10ms);
    }
  }

  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::atomic<bool> running_{true};
  std::thread worker_;
};

class ExecutorThread final {
 public:
  explicit ExecutorThread(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
      : executor_(std::move(executor)), worker_([this] { executor_->spin(); }) {}

  ~ExecutorThread() {
    executor_->cancel();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread worker_;
};

TEST(ControllerManagerIntegration, ExecutesActionTrajectoryAsAtomicBoardCommands) {
  RclcppScope rclcpp_scope;
  ssc32_ros2_control::test::Ssc32Emulator emulator({{0U, 1500U}, {1U, 1500U}});
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<controller_manager::ControllerManager>(
      executor, example_urdf(emulator.device()), true, "ssc32_test_controller_manager");
  executor->add_node(manager);

  const std::string parameters =
      std::string(SSC32_ROS2_CONTROL_SOURCE_DIR) + "/test/support/arm_controller.yaml";
  manager->set_parameter(rclcpp::Parameter("arm_controller.params_file", parameters));
  manager->set_parameter(rclcpp::Parameter(
      "arm_controller.type", "joint_trajectory_controller/JointTrajectoryController"));
  const auto controller = manager->load_controller(
      "arm_controller", "joint_trajectory_controller/JointTrajectoryController");
  ASSERT_NE(controller, nullptr);
  ASSERT_EQ(manager->configure_controller("arm_controller"), controller_interface::return_type::OK);

  ControlLoop control_loop(manager);
  ASSERT_EQ(
      manager->switch_controller({"arm_controller"}, {},
                                 controller_manager_msgs::srv::SwitchController::Request::STRICT),
      controller_interface::return_type::OK);

  auto client_node = std::make_shared<rclcpp::Node>("ssc32_trajectory_test_client");
  executor->add_node(client_node);
  ExecutorThread executor_thread(executor);
  auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      client_node, "/arm_controller/follow_joint_trajectory");
  ASSERT_TRUE(action_client->wait_for_action_server(2s));

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"base_joint", "shoulder_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  // Both targets map to multiples of QP's 10 us reporting resolution.
  point.positions = {0.1, -0.04};
  point.time_from_start = rclcpp::Duration(1s);
  goal.trajectory.points.push_back(point);
  goal.goal_time_tolerance = rclcpp::Duration(1s);

  const auto goal_future = action_client->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(2s), std::future_status::ready);
  const auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  const auto result_future = action_client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(result.result, nullptr);
  EXPECT_EQ(result.result->error_code, FollowJointTrajectory::Result::SUCCESSFUL);

  ASSERT_TRUE(emulator.wait_for_move_count(2U, 1s));
  const auto moves = emulator.move_commands();
  for (const auto& command : moves) {
    EXPECT_NE(command.find("#0 P"), std::string::npos);
    EXPECT_NE(command.find("#1 P"), std::string::npos);
  }
  EXPECT_EQ(emulator.pulse(0U), 1550U);
  EXPECT_EQ(emulator.pulse(1U), 1520U);
  EXPECT_TRUE(emulator.error().empty()) << emulator.error();
}

}  // namespace
