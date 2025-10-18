cmake_minimum_required(VERSION 3.8)
project(edrom_lowlevel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(aurea_walk REQUIRED)


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

add_executable(head_reader src/head_reader.cpp)

ament_target_dependencies(head_reader
  rclcpp
  sensor_msgs
  dynamixel_sdk
)

add_executable(robot_initializer src/robot_initializer.cpp)
ament_target_dependencies(robot_initializer
  rclcpp
  sensor_msgs
  geometry_msgs
  aurea_walk
)

ament_export_dependencies(pluginlib)

install(TARGETS
  direct_controller
  robot_initializer
  head_reader
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
