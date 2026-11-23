# Contributing

Keep protocol behavior in the `ssc32` library and ROS lifecycle behavior here.
Changes should include deterministic tests that do not require physical
hardware. Hardware reports should state controller model and firmware, USB
serial adapter, baud rate, ROS distribution, and the smallest safe reproduction.

Run before submitting:

```sh
colcon build --packages-select-up-to ssc32_ros2_control \
  --cmake-args -DSSC32_ROS2_CONTROL_WARNINGS_AS_ERRORS=ON
colcon test --packages-select ssc32 ssc32_ros2_control
colcon test-result --verbose
```
