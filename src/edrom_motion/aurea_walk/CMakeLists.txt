cmake_minimum_required(VERSION 3.8)
project(aurea_walk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveIK.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

add_library(ik_solver src/ik_solver.cpp)
target_include_directories(ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(ik_solver PUBLIC Eigen3::Eigen)

add_executable(ik_node src/ik_node.cpp)
target_include_directories(ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ik_node
  rclcpp
  geometry_msgs
  sensor_msgs
)
target_link_libraries(ik_node
  ik_solver
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

add_executable(walking_engine_node src/walking_engine_node.cpp src/trajectory_generator.cpp)
target_include_directories(walking_engine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(walking_engine_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
)
target_link_libraries(walking_engine_node
  Eigen3::Eigen
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS
  ik_node
  walking_engine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ik_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dynamixel_node.py
    scripts/motion_controller_node.py
    scripts/planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/ launch/ srv/ configs/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
