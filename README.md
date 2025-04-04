# ARDRONE ROS2

ROS2 nodes for JumpingSumo (and sample for Bebop)

## Features

- Control JumpingSumo robot via ROS2 topics
  - Subscribe to geometry_msgs/Twist on `jumpingsumo/cmd_vel` for motion control
  - Publish sensor_msgs/Image on `jumpingsumo/image_raw` for video feed
  - Publish sensor_msgs/BatteryState on `jumpingsumo/battery` for battery status
  - WiFi connection handling

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y --ignore-src
colcon build --packages-up-to ardrone_ros --event-handlers console_direct+
```

## Usage

1. Connect to JumpingSumo's WiFi network
2. Launch the ROS2 node:
```bash
ros2 run ardrone_ros jumping_sumo
```

### Debug Logging

To enable debug logging for sensor detection and detailed command information:

```bash
ros2 run ardrone_ros jumping_sumo --ros-args --log-level jumping_sumo:=debug
```

This will show all DEBUG level messages, which include:
- Detailed command key information
- All sensor data values
- IMU data when available
- Unknown command arguments

For even more verbose logging, you can set the ROS_LOG_MIN_SEVERITY environment variable:

```bash
export ROS_LOG_MIN_SEVERITY=DEBUG
ros2 run ardrone_ros jumping_sumo
```
