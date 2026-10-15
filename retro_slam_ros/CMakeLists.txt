# MIT License

# Copyright (c) 2026 Saurabh Gupta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
cmake_minimum_required(VERSION 3.22)
project(retro_slam_ros VERSION 0.0.1 LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/ AND NOT IS_SYMLINK ${CMAKE_CURRENT_SOURCE_DIR})
  message(STATUS "FOUND RETRO-SLAM in source tree, using it instead of fetching from Github")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp ${CMAKE_CURRENT_BINARY_DIR}/retro_slam)
else()
  message(STATUS "Performing out-of-tree build, fetching RETRO-SLAM v${CMAKE_PROJECT_VERSION} Release from Github")
  include(FetchContent)
  FetchContent_Declare(ext_retro_slam_core PREFIX retro_slam_core GIT_REPOSITORY git@github.com:PRBonn/retro-slam.git
                       GIT_TAG main SOURCE_SUBDIR cpp/)
  FetchContent_MakeAvailable(ext_retro_slam_core)
endif()

# Build
find_package(ament_cmake REQUIRED)

# Core ROS
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)

# Messages
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

function(configure_target target)
  target_compile_features(${target} PRIVATE cxx_std_20)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endfunction()

set(COMMON_ROS_DEPS
    rclcpp
    tf2_ros
    tf2_eigen
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs)

# ROS 2 node
add_executable(odometry_node ${CMAKE_CURRENT_SOURCE_DIR}/src/OdometryNode.cpp)
configure_target(odometry_node)
ament_target_dependencies(odometry_node ${COMMON_ROS_DEPS})
target_link_libraries(odometry_node kiss_icp_core kiss_icp_pipeline retro_slam_core)

add_executable(mapping_node ${CMAKE_CURRENT_SOURCE_DIR}/src/MappingNode.cpp)
configure_target(mapping_node)
ament_target_dependencies(mapping_node ${COMMON_ROS_DEPS} rclcpp_components message_filters example_interfaces)
target_link_libraries(mapping_node retro_slam_core)

include(GNUInstallDirs)
install(TARGETS odometry_node mapping_node LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
