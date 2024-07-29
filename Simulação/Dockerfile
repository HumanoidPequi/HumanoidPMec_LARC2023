# Use a imagem base do ROS Noetic com suporte a GPU
FROM ros:noetic-ros-core

# Instale dependências necessárias
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    build-essential \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Instale o pacote ROS desejado
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-ros-control \
    ros-$ROS_DISTRO-ros-controllers \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-rviz \
    && rm -rf /var/lib/apt/lists/*


# Copiar o diretório pacotes da Martha
RUN mkdir -p /root/Martha_ws/src
COPY Martha_ws/src /root/Martha_ws/src

# Construir o workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /root/Martha_ws && catkin_make"

# Fazer o source do setup.bash do workspace
RUN echo "source /root/Martha_ws/devel/setup.bash" >> ~/.bashrc

# So pra usuarios de Windows
# RUN echo "export GAZEBO_IP=127.0.0.1" >> ~/.bashrc
# RUN echo "export DISPLAY=host.docker.internal:0" >> ~/.bashrc
# RUN echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc

# Definir o comando padrão
CMD ["bash"]
