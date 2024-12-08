# ARDRONE ROS2

ROS2 nodes for JumpingSumo (and sample for Bebop)

## Features

- Control JumpingSumo robot via ROS2 topics
  - Subscribe to geometry_msgs/Twist on `jumpingsumo/cmd_vel` for motion control
  - Publish sensor_msgs/Image on `jumpingsumo/raw_image` for video feed
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
