cmake_minimum_required(VERSION 3.22)
project(lbr_ros2_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fri_vendor REQUIRED)
find_package(FRIClient REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kinematics_interface_kdl REQUIRED)
find_package(lbr_fri_msgs REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

# LBR ROS 2 control library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/lbr_estimated_ft_broadcaster.cpp
  src/lbr_state_broadcaster.cpp
  src/lbr_system_interface.cpp
)

# Add include directories
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

# Link against dependencies
ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  Eigen3
  fri_vendor
  geometry_msgs
  hardware_interface
  kinematics_interface_kdl
  lbr_fri_msgs
  lbr_fri_ros2
  pluginlib
  rclcpp
  realtime_tools
)

target_link_libraries(${PROJECT_NAME}
  FRIClient::FRIClient
)

pluginlib_export_plugin_description_file(controller_interface lbr_broadcasters.xml)
pluginlib_export_plugin_description_file(hardware_interface lbr_system_interface.xml)

# Export for downstream usage, see https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html
ament_export_targets(
  ${PROJECT_NAME}_export HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  controller_interface
  eigen3_cmake_module
  Eigen3
  fri_vendor
  FRIClient
  geometry_msgs
  hardware_interface
  kinematics_interface_kdl
  lbr_fri_msgs
  lbr_fri_ros2
  pluginlib
  rclcpp
  realtime_tools
)
  
install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_export
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Launch mixins
ament_python_install_package(${PROJECT_NAME})

ament_package()
