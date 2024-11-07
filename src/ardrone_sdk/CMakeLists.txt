cmake_minimum_required(VERSION 3.8)
project(ardrone_sdk)

include(ExternalProject)
ExternalProject_Add(
  ardrone-sdk
  URL https://github.com/vtalpaert/ardrone-sdk-native/releases/download/arsdk-native-samples-x64-master/arsdk-native-samples-x64-master.tar.gz
  PREFIX sdk
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)
# could use ament_download, but ExternalProject will unzip archive

set(SDK_SRC ${CMAKE_CURRENT_BINARY_DIR}/sdk/src/ardrone-sdk/usr)

install(
  DIRECTORY ${SDK_SRC}/bin ${SDK_SRC}/include ${SDK_SRC}/lib ${SDK_SRC}/share
  DESTINATION .
  USE_SOURCE_PERMISSIONS)

set(LIBRARY_NAME ${PROJECT_NAME})

# Use ament to declare a library
find_package(ament_cmake REQUIRED)

# Create empty target
add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(
  ${LIBRARY_NAME}
  INTERFACE
  ${CMAKE_INSTALL_PREFIX}/include/)
FILE(GLOB SDK_LIBS ${CMAKE_INSTALL_PREFIX}/lib/*.so ${CMAKE_INSTALL_PREFIX}/lib/*.a)
target_link_libraries(${LIBRARY_NAME} INTERFACE ${SDK_LIBS})
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION 3.14.0)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

## ament macros generate the libConfig.cmake
# Announce there is a lib/ folder installed
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# Mark the directory of the exported include directories
ament_export_include_directories("include")

# Setup project
ament_package()