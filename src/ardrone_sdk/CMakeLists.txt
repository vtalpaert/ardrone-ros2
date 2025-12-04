cmake_minimum_required(VERSION 3.12)
project(ardrone_sdk VERSION 3.14.0 LANGUAGES C)

# Use ament to declare a package
find_package(ament_cmake REQUIRED)

# List all private headers and sources
#file(GLOB_RECURSE ARSDK_SRC
#	"src/*.h"
#    "src/*.c"	
#)
file(GLOB_RECURSE ARSDK_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/Sources/*"
)

# Remove the implementation files that are included by other files
list(REMOVE_ITEM ARSDK_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/Sources/pomp_loop_linux.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Sources/pomp_loop_posix.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Sources/pomp_loop_win32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Sources/pomp_timer_linux.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Sources/pomp_timer_posix.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Sources/ARDISCOVERY_AvahiDiscovery_nodbus.c"
)

# Create ardrone_sdk as a library
add_library(${PROJECT_NAME}_lib SHARED ${ARSDK_SRC})
#add_library(${PROJECT_NAME}::${PROJECT_NAME}_lib ALIAS ${PROJECT_NAME}_lib)

## Headers

# Public headers (only the ones in include/)
file(GLOB_RECURSE ARSDK_PUBLIC_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*"
)
# Set public headers for the target
#target_sources(${PROJECT_NAME}_lib PUBLIC ${ARSDK_PUBLIC_INCLUDE})
#set_target_properties(${PROJECT_NAME}_lib PROPERTIES PUBLIC_HEADER ${ARSDK_PUBLIC_INCLUDE})

# include/ contains the public headers
# Includes/ contains all headers (including generated headers)
# Sources/ contains private headers
target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Includes>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Sources>
)

# Avoid redefining mmsghdr already in /usr/include/x86_64-linux-gnu/sys/socket.h
# The flag is in Sources/arstream2_rtp.h
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE HAS_MMSG)

# Math, Avahi and JSON-C
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONC REQUIRED json-c)
pkg_check_modules(AVAHI REQUIRED avahi-client)
target_link_libraries(${PROJECT_NAME}_lib PRIVATE
    m
    ${JSONC_LIBRARIES}
    ${AVAHI_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_lib PRIVATE
    ${AVAHI_INCLUDE_DIRS}
    ${JSONC_INCLUDE_DIRS}
)

# Minimal executable to verify includes and linking
if(BUILD_TESTING)
    add_executable(test_compile ${CMAKE_CURRENT_SOURCE_DIR}/test/test_compile.c)
    target_include_directories(test_compile
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_link_libraries(test_compile PUBLIC ${PROJECT_NAME}_lib)
endif()

## Samples
# JumpingSumoSample
find_package(Curses REQUIRED)
add_executable(JumpingSumoSample
    ${CMAKE_CURRENT_SOURCE_DIR}/JumpingSumoSample/ihm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/JumpingSumoSample/JumpingSumoSample.c
)
target_include_directories(JumpingSumoSample
    PRIVATE
    ${CURSES_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/JumpingSumoSample/>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(JumpingSumoSample PUBLIC
    ${CURSES_LIBRARIES}
    ${PROJECT_NAME}_lib
)

install(
    TARGETS ${PROJECT_NAME}_lib JumpingSumoSample
    EXPORT export_${PROJECT_NAME}
    INCLUDES DESTINATION include
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(
    DIRECTORY include/
    DESTINATION include
)
install(
    FILES LICENSE.md
    DESTINATION .
)
#install(EXPORT export_${PROJECT_NAME}
#    FILE ${PROJECT_NAME}Targets.cmake
#    NAMESPACE ${PROJECT_NAME}::
#    DESTINATION lib/cmake/${PROJECT_NAME}
#)

# Configure ament to include the targets file
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Setup project
ament_package()
