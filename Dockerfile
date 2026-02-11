FROM ros:jazzy

ARG USERNAME=milo

# Install Java + ROS build tools (no rosdep init)
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    openjdk-21-jdk \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    ros-jazzy-ament-cmake \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && useradd -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Source ROS automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc

# Placeholder for cloning repos
WORKDIR /home/$USERNAME/ws/src
RUN git clone https://github.com/PaoloForte95/navigo.git
RUN git clone https://github.com/PaoloForte95/common_interfaces.git
WORKDIR /home/$USERNAME/ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && colcon build --packages-select navigo location_msgs material_handler_msgs object_detection_msgs standard_msgs

RUN echo "source /home/$USERNAME/ws/install/setup.bash" >> /home/$USERNAME/.bashrc

WORKDIR /home/$USERNAME/ws/src
COPY ./ ./atlantis
SHELL ["/bin/bash", "-c"]
WORKDIR /home/$USERNAME/ws
RUN source /home/$USERNAME/ws/install/setup.bash && colcon build --packages-up-to atlantis && source /home/$USERNAME/ws/install/setup.bash

WORKDIR /home/$USERNAME/ws

USER $USERNAME

CMD ["/bin/bash"]

