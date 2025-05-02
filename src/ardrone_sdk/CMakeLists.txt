cmake_minimum_required(VERSION 3.12)
project(ardrone_sdk)

set(LIBRARY_NAME ${PROJECT_NAME})

# Use ament to declare a library
find_package(ament_cmake REQUIRED)

# Create interface target to existing files already install in ardrone_sdk folder by arsdk package
add_library(${LIBRARY_NAME} INTERFACE)
# The library version is dependent on the actual downloaded library in arsdk3
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION 3.14.0)

# Add the include dir
target_include_directories(
  ${LIBRARY_NAME}
  INTERFACE
  $<INSTALL_INTERFACE:include/>
)

# Add individually listed shared library files
# Since we cannot use a folder, we need to GLOB over each file in the lib folder.
# But the arsdk archive is only downloaded at INSTALL time, which is why we create
# an almost empty separate ROS package to ExternalProject_Add the compiled lib
file(GLOB SDK_LIBS RELATIVE ${CMAKE_INSTALL_PREFIX} ${CMAKE_INSTALL_PREFIX}/lib/*.so)
list(SORT SDK_LIBS)
list(TRANSFORM SDK_LIBS PREPEND "$\{_IMPORT_PREFIX\}/")
target_link_libraries(${LIBRARY_NAME} INTERFACE ${SDK_LIBS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Clean up sdk folder, since `source install/setup.bash` will complain for the empty arsdk3 folder
#install(CODE "message(STATUS \"Removing: ${CMAKE_INSTALL_PREFIX}/../arsdk3\")")
#install(CODE "file(REMOVE_RECURSE ${CMAKE_INSTALL_PREFIX}/../arsdk3)")
# ! Commented since colcon will search for an existing arsdk3 folder

# Setup project
ament_package()
