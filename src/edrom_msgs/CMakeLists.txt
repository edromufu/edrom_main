cmake_minimum_required(VERSION 3.8)
project(edrom_msgs)

# 1. ENCONTRAR TODOS OS PACOTES NECESSÁRIOS
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ADIÇÃO: Encontre os pacotes dos quais suas mensagens dependem.
# A maioria dos pacotes de mensagem depende de std_msgs para coisas como o Header.
find_package(std_msgs REQUIRED)


# 2. GERAR AS INTERFACES (MENSAGENS)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/VisionData.msg"
  "msg/LandmarkArray.msg"
  "msg/Landmark.msg"
  "msg/IMU.msg"
  "msg/HeadMovement.msg"
  # ADIÇÃO: Declare as dependências aqui.
  DEPENDENCIES std_msgs
)


# 3. EXPORTAR DEPENDÊNCIAS E INSTALAR
# Esta seção informa a outros pacotes como usar suas mensagens.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ADIÇÃO: Exporta as dependências para que outros pacotes saibam do que precisam.
ament_export_dependencies(rosidl_default_runtime)

ament_package()
