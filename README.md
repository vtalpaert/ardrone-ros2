# ARDRONE ROS2

ROS2 nodes for JumpingSumo (and sample for Bebop)

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y --ignore-src
colcon build --packages-up-to ardrone_ros --event-handlers console_direct+
```
