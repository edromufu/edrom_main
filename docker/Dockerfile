# Nome da Imagem Base do ROS2 (Ubuntu 22.04 LTS para Humble)
FROM osrf/ros:humble-desktop-full-jammy

# --- Metadados e Definições ---
LABEL maintainer="Pedro Henrique B. Peres<pedrohperescode@gmail.com>"
LABEL description="ROS2 Humble development environment for EDROM"

# --- Configurações Locais e Linguagem ---
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# --- Variáveis de Ambiente do ROS2 ---
ENV ROS_DISTRO humble
ENV ROS_DOMAIN_ID 0
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# --- Ferramentas de Desenvolvimento Comuns ---
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    net-tools \
    iputils-ping \
    nano \
    vim \
    # --- NOVO: Garante que o CycloneDDS esteja instalado ---
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

# --- Criação e Configuração do Usuário ---
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} devuser && useradd -m -s /bin/bash -u ${UID} -g devuser devuser
RUN usermod -aG sudo devuser

# Personaliza o .bashrc do devuser para carregar o ambiente ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> /home/devuser/.bashrc
RUN echo "source /workspace/install/setup.bash" >> /home/devuser/.bashrc
RUN echo "alias cbr='colcon build --symlink-install && source install/setup.bash'" >> /home/devuser/.bashrc

# Define o usuário padrão para futuras operações
USER devuser

# --- Ponto de Entrada para o Shell ROS2 ---
WORKDIR /workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]
