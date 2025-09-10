cmake_minimum_required(VERSION 3.8)
project(edrom_lowlevel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(direct_controller src/direct_controller.cpp)
target_include_directories(direct_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(direct_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  dynamixel_sdk
)

add_library(aurea_hardware_interface SHARED
  src/aurea_hardware_interface.cpp
)
target_include_directories(aurea_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(aurea_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  dynamixel_sdk
)

ament_export_dependencies(pluginlib)

install(TARGETS
  aurea_hardware_interface
  direct_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()