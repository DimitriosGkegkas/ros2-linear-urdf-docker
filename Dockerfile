FROM ubuntu:22.04 AS base

# Set non-interactive mode for apt-get
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# Installing ros-humble-desktop based from here https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
RUN apt-get update && \
    apt-get install locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

RUN apt update && \
    apt upgrade -y && \
    apt install -y ros-humble-desktop  && \
    apt install -y ros-dev-tools

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libyaml-cpp-dev \
    ros-humble-rviz-visual-tools

RUN apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-rosbridge-server \
    ros-humble-xacro

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws