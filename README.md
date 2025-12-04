# ARDRONE ROS2

Exports ARDRONE ARSDK3 from Parrot as a ROS2 package. A node for the Jumping Sumo is provided, but more nodes are welcome in PR.

## ARDRONE_SDK

### Build the ROS SDK

```bash
source /opt/ros/humble/setup.bash
rosdep install -i --from-path src --rosdistro humble -y --ignore-src
colcon build --packages-up-to ardrone_sdk --event-handlers console_direct+ --paths src/*
```

`ardrone_sdk` exports headers and libs for easy integration, as illustrated in `ardrone_sumo`

```cmake
find_package(ardrone_sdk REQUIRED)
add_executable(jumping_sumo src/jumping_sumo.cpp)
ament_target_dependencies(jumping_sumo 
  ardrone_sdk::ardrone_sdk_lib
)
```

### Development

The [original ARSDK](https://github.com/Parrot-Developers/arsdk_manifests) build involves using `repo init`/`repo sync` to clone the source files from github. To avoid cloning in the ROS build farm, we re-commit the files in the `src/ardrone_sdk` package and preserve the Parrot license. Only a part of the original files is committed, which is controlled by `./scripts/sync_sources.sh`. Edit this script and commit the new output to add more ARSDK libraries.

The Bebop Sample is not part of this release.

## ARDRONE_SUMO

![Jumping Sumo](docs/parrot-minidrone-jumping-sumo.jpg)
ROS2 node for Jumping Sumo.

### Features

Control the JumpingSumo drone via ROS2 topics

- Subscribe to geometry_msgs/Twist on `jumpingsumo/cmd_vel` for motion control
- Publish sensor_msgs/Image on `jumpingsumo/image_raw` for video feed
- Publish sensor_msgs/BatteryState on `jumpingsumo/battery` for battery status
- WiFi connection handling

### Usage

1. Connect to JumpingSumo's WiFi network
2. Launch the ROS2 node:

```bash
ros2 run ardrone_sumo jumping_sumo
```

To control the drone, use:

```bash
# Terminal 1
ros2 run rqt_image_view rqt_image_view /jumpingsumo/image_raw
# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/jumpingsumo/cmd_vel
```

The camera distortion is apparent for nearby objects, so calibration is needed for further use

![nearby](docs/penguin_close.png)
![further](docs/penguin_less_close.png)

The original sample may still be run

```bash
source install/setup.bash
JumpingSumoSample
```

To enable debug logging for detailed command information:

```bash
ros2 run ardrone_sumo jumping_sumo --ros-args --log-level jumping_sumo:=debug
```

This will show all DEBUG level messages, which include detailed command key information
