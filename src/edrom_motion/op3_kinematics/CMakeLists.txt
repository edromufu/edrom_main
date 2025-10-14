cmake_minimum_required(VERSION 3.8)
project(op3_kinematics)

# 1. Encontra todos os pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_python REQUIRED)
# 2. Gera as interfaces do serviço
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveIK.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# 3. Compila a biblioteca da IK
add_library(ik_solver_lib src/ik_solver.cpp)
target_include_directories(ik_solver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(ik_solver_lib PUBLIC Eigen3::Eigen)

# 4. Compila o nó executável do serviço
add_executable(ik_node src/ik_node.cpp)
ament_target_dependencies(ik_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Linkamos explicitamente com a biblioteca de "type support" C++
# que foi gerada pelo rosidl. Esta é a forma mais direta de resolver o erro
# de "undefined reference".
target_link_libraries(ik_node
  ik_solver_lib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 6. Declaração do pacote e instalação
install(
  PROGRAMS
    scripts/dynamixel_node.py
    scripts/motion_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
install(
  DIRECTORY include/ launch/ srv/
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ik_solver_lib ik_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)