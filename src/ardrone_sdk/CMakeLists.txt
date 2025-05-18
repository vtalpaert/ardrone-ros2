cmake_minimum_required(VERSION 3.12)
project(ardrone_sdk)


# Start with downloading the ARSDK3 compiled SDK for linux
include(ExternalProject)
ExternalProject_Add(
	arsdk3
	URL https://github.com/vtalpaert/ardrone-sdk-native/releases/download/arsdk-native-samples-x64-master/arsdk-native-samples-x64-master.tar.gz
	URL_MD5 03a38db362392bfa8576d9de64f2440a
	PREFIX arsdk3
	DOWNLOAD_NO_PROGRESS true
	CONFIGURE_COMMAND ""
	BUILD_COMMAND ""
	INSTALL_COMMAND ""
)

# Install the library into ardrone_sdk
set(SDK_SRC ${CMAKE_CURRENT_BINARY_DIR}/arsdk3/src/arsdk3/usr)
set(SDK_LIC ${CMAKE_CURRENT_BINARY_DIR}/arsdk3/src/arsdk3/parrot_license.md)
install(
	DIRECTORY ${SDK_SRC}/bin ${SDK_SRC}/include ${SDK_SRC}/lib ${SDK_SRC}/share
	DESTINATION .
	USE_SOURCE_PERMISSIONS
	MESSAGE_NEVER
)
install(FILES ${SDK_LIC} DESTINATION .)

# Use ament to declare a package
find_package(ament_cmake REQUIRED)

set(LIBRARY_NAME ${PROJECT_NAME})
# Create interface target to existing files already installed in ardrone_sdk folder by arsdk3 ExternalProject
add_library(${LIBRARY_NAME} INTERFACE)

# The library version is dependent on the actual downloaded library in arsdk3
# Here we will hardcode the version
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION 3.14.0)

# Add the include dir
target_include_directories(
	${LIBRARY_NAME}
	INTERFACE
	$<INSTALL_INTERFACE:include/>
)

# Add individually listed shared library files
# The arsdk3 archive is only downloaded at INSTALL time, so we cannot GLOB
# over the lib folder. Instead we hardcode the list of libraries.
# The libraries will be linked when using find_package(ardrone_sdk REQUIRED)
# thanks to the install/ardrone_sdk/share/ardrone_sdk/cmake/export_ardrone_sdkExport.cmake
# file
set(INTERFACE_LINK_LIBRARIES "$\{_IMPORT_PREFIX\}/lib/libarcommands.so;$\{_IMPORT_PREFIX\}/lib/libarcontroller.so;$\{_IMPORT_PREFIX\}/lib/libardatatransfer.so;$\{_IMPORT_PREFIX\}/lib/libardiscovery.so;$\{_IMPORT_PREFIX\}/lib/libarmavlink.so;$\{_IMPORT_PREFIX\}/lib/libarmedia.so;$\{_IMPORT_PREFIX\}/lib/libarnetwork.so;$\{_IMPORT_PREFIX\}/lib/libarnetworkal.so;$\{_IMPORT_PREFIX\}/lib/libarsal.so;$\{_IMPORT_PREFIX\}/lib/libarstream.so;$\{_IMPORT_PREFIX\}/lib/libarstream2.so;$\{_IMPORT_PREFIX\}/lib/libarupdater.so;$\{_IMPORT_PREFIX\}/lib/libarutils.so;$\{_IMPORT_PREFIX\}/lib/libcrypto.so;$\{_IMPORT_PREFIX\}/lib/libcurl.so;$\{_IMPORT_PREFIX\}/lib/libfutils.so;$\{_IMPORT_PREFIX\}/lib/libjson-c.so;$\{_IMPORT_PREFIX\}/lib/libjson.so;$\{_IMPORT_PREFIX\}/lib/libmux.so;$\{_IMPORT_PREFIX\}/lib/libpomp.so;$\{_IMPORT_PREFIX\}/lib/libpuf.so;$\{_IMPORT_PREFIX\}/lib/librtsp.so;$\{_IMPORT_PREFIX\}/lib/libsdp.so;$\{_IMPORT_PREFIX\}/lib/libssl.so;$\{_IMPORT_PREFIX\}/lib/libtar.so;$\{_IMPORT_PREFIX\}/lib/libtls.so;$\{_IMPORT_PREFIX\}/lib/libulog.so")
target_link_libraries(${LIBRARY_NAME} INTERFACE ${INTERFACE_LINK_LIBRARIES})
ament_export_targets(export_${LIBRARY_NAME} HAS_LIBRARY_TARGET)

install(
	TARGETS ${LIBRARY_NAME}
	EXPORT export_${LIBRARY_NAME}
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
