  cmake_minimum_required(VERSION 3.8)
  project(aurea_kick)

  # 1. Encontrar pacotes
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(aurea_walk REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(Eigen3 REQUIRED)

  # 2. Gerar a interface da Ação
  rosidl_generate_interfaces(${PROJECT_NAME}
    "action/Kick.action"
    DEPENDENCIES action_msgs geometry_msgs
  )
  add_executable(teleop_kick_bridge src/teleop_kick_bridge.cpp)
  target_include_directories(teleop_kick_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 2. Adiciona as dependências de pacotes ROS
ament_target_dependencies(teleop_kick_bridge
  rclcpp
  rclcpp_action
  std_msgs
)

# 3. Linka com a biblioteca da Ação gerada (a correção principal)
target_link_libraries(teleop_kick_bridge
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
  # 3. Construir a biblioteca da engine
  add_library(kick_engine src/kick_engine.cpp)
  target_include_directories(kick_engine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS}
  )
  target_link_libraries(kick_engine PUBLIC Eigen3::Eigen)
  # 4. Construir o nó executável
  add_executable(kick_node src/kick_node.cpp)
  # Adiciona os diretórios de include para que o nó encontre seus próprios headers
  target_include_directories(kick_node PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  # Adiciona as dependências de pacotes ROS
  ament_target_dependencies(kick_node
    rclcpp
    rclcpp_action
    aurea_walk
  )
  # Linka com a engine, Eigen e, mais importante, com a Ação gerada
  target_link_libraries(kick_node
    kick_engine
    Eigen3::Eigen
    ${PROJECT_NAME}__rosidl_typesupport_cpp
  )

  # 5. Instalação
  install(TARGETS
    kick_engine
    kick_node
    teleop_kick_bridge
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(
    DIRECTORY include/ launch/ config/ action/
    DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )



  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
  )
  # 6. Finalização
  ament_package()