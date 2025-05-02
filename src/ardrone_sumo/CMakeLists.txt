cmake_minimum_required(VERSION 3.8)
project(ardrone_sumo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ardrone_sdk REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(jumping_sumo src/jumping_sumo.cpp)
ament_target_dependencies(jumping_sumo 
  rclcpp
  sensor_msgs
  ardrone_sdk
  OpenCV
  cv_bridge
)

install(TARGETS
  jumping_sumo
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
