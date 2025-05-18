^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ardrone_sumo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-18)
------------------
* [kilted] Update deprecated call to ament_target_dependencies (`#1 <https://github.com/vtalpaert/ardrone-ros2/issues/1>`_)
* remove BUILD_TESTING CMake condition, since there are no tests
* Contributors: David V. Lu!!, Victor Talpaert

1.0.0 (2025-05-02)
------------------
* ROS2 node to control the Jumping Sumo Parrot drone
* Connect over private WiFi to retrieve the video feed, battery level and send motor commands
* No IMU data, no standard units for motor commands (currently in percentage of maximum value)
* Contributors: Victor Talpaert
