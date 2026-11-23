cmake_minimum_required(VERSION 3.16)

project(ssc32_ros2_control VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ssc32 CONFIG REQUIRED)

option(SSC32_ROS2_CONTROL_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

add_library(ssc32_ros2_control SHARED
  src/driver.cpp
  src/joint_calibration.cpp
  src/ssc32_system.cpp
)
target_compile_features(ssc32_ros2_control PUBLIC cxx_std_17)
target_include_directories(ssc32_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ssc32_ros2_control PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ssc32::ssc32
)

function(ssc32_ros2_control_enable_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
      $<$<BOOL:${SSC32_ROS2_CONTROL_WARNINGS_AS_ERRORS}>:-Werror>
    )
  endif()
endfunction()

ssc32_ros2_control_enable_warnings(ssc32_ros2_control)
pluginlib_export_plugin_description_file(hardware_interface ssc32_ros2_control.xml)

install(TARGETS ssc32_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})
install(FILES README.md ARCHITECTURE.md TESTING.md ssc32.repos DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_calibration test/test_calibration.cpp)
  target_link_libraries(test_calibration ssc32_ros2_control)
  ssc32_ros2_control_enable_warnings(test_calibration)

  ament_add_gtest(test_driver test/test_driver.cpp)
  target_link_libraries(test_driver ssc32_ros2_control)
  ssc32_ros2_control_enable_warnings(test_driver)

  ament_add_gtest(test_plugin test/test_plugin.cpp)
  target_compile_definitions(test_plugin PRIVATE
    SSC32_ROS2_CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(test_plugin
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    ssc32_ros2_control
  )
  ssc32_ros2_control_enable_warnings(test_plugin)

  if(UNIX)
    find_package(control_msgs REQUIRED)
    find_package(controller_manager REQUIRED)
    find_package(controller_manager_msgs REQUIRED)
    find_package(rclcpp_action REQUIRED)
    find_package(trajectory_msgs REQUIRED)

    add_library(ssc32_test_emulator STATIC test/support/ssc32_emulator.cpp)
    target_compile_features(ssc32_test_emulator PUBLIC cxx_std_17)
    target_include_directories(ssc32_test_emulator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/test)
    ssc32_ros2_control_enable_warnings(ssc32_test_emulator)

    ament_add_gtest(test_pty test/test_pty.cpp)
    target_link_libraries(test_pty ssc32_ros2_control)
    ssc32_ros2_control_enable_warnings(test_pty)

    ament_add_gtest(test_system_pty test/test_system_pty.cpp)
    target_compile_definitions(test_system_pty PRIVATE
      SSC32_ROS2_CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_link_libraries(test_system_pty
      hardware_interface::hardware_interface
      rclcpp::rclcpp
      ssc32_ros2_control
      ssc32_test_emulator
    )
    ssc32_ros2_control_enable_warnings(test_system_pty)

    ament_add_gtest(test_controller_manager test/test_controller_manager.cpp)
    target_compile_definitions(test_controller_manager PRIVATE
      SSC32_ROS2_CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_link_libraries(test_controller_manager
      control_msgs::control_msgs
      controller_manager::controller_manager
      controller_manager_msgs::controller_manager_msgs
      rclcpp::rclcpp
      rclcpp_action::rclcpp_action
      ssc32_ros2_control
      ssc32_test_emulator
      trajectory_msgs::trajectory_msgs
    )
    ssc32_ros2_control_enable_warnings(test_controller_manager)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(ssc32_ros2_control)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle ssc32)
ament_package()
