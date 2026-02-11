FROM ros:humble
ARG USERNAME=milo
ARG USER_UID=1000
ARG USER_GID=$USER_UID


# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get install -y python3-pip libopenjp2-7 libopenjp2-7-dev libcups2-dev
ENV SHELL /bin/bash


# Install ros2 humble
RUN DEBIAN_FRONTEND=noninteractive apt install -yq ros-humble-desktop xterm nano gedit ros-humble-xacro ros-humble-teleop-twist-keyboard ros-humble-robot-state-publisher ros-humble-ros2-controllers ros-humble-geographic-msgs ros-humble-launch-param-builder ros-humble-nav2-msgs


# Copy the atlantis fiels
WORKDIR /home/milo/ws/src
COPY ./ ./atlantis

RUN git clone https://github.com/PaoloForte95/material_handler
#RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
# source ros2 and workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /home/milo/.bashrc

WORKDIR /home/milo/ws
# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME


CMD ["/bin/bash"]