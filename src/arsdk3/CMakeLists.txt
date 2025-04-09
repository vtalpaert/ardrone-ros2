cmake_minimum_required(VERSION 3.8)
project(arsdk3)

# Put the bin include lib and share folders in SDK_DESTINATION
# Note that to publish the package, we might need to reconsider the current path
set(SDK_DESTINATION ../ardrone_sdk)
set(SDK_SRC ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/src/${PROJECT_NAME}/usr)
set(SDK_LIC ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/src/${PROJECT_NAME}/parrot_license.md)

include(ExternalProject)
ExternalProject_Add(
  ${PROJECT_NAME}
  URL https://github.com/vtalpaert/ardrone-sdk-native/releases/download/arsdk-native-samples-x64-master/arsdk-native-samples-x64-master.tar.gz
  URL_MD5 03a38db362392bfa8576d9de64f2440a
  PREFIX ${PROJECT_NAME}
  DOWNLOAD_NO_PROGRESS true
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

install(
  DIRECTORY ${SDK_SRC}/bin ${SDK_SRC}/include ${SDK_SRC}/lib ${SDK_SRC}/share
  DESTINATION ${SDK_DESTINATION}
  USE_SOURCE_PERMISSIONS
  MESSAGE_NEVER
)

install(FILES ${SDK_LIC} DESTINATION ${SDK_DESTINATION})

find_package(ament_cmake REQUIRED)
ament_package()
