ARG ROS_DISTRO=humble
ARG PACKAGE=ardrone_sumo
FROM osrf/ros:${ROS_DISTRO}-desktop

SHELL [ "/bin/bash" , "-c" ]

#RUN sudo apt update \
#    && sudo apt upgrade -y \
#    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY src src/

RUN apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --packages-up-to ${PACKAGE}

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
