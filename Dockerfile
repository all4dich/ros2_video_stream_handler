FROM ubuntu:22.04

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Set up locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Enable Ubuntu Universe repository
RUN apt-get install -y software-properties-common
RUN add-apt-repository universe

# Add ROS2 repository
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install colcon
RUN apt-get update && apt-get install -y python3-colcon-common-extensions

# Install ROS2 Humble Desktop
RUN apt-get update && apt-get install -y ros-humble-desktop
RUN apt-get update && apt-get install -y ros-humble-ros-base
RUN apt-get update && apt-get install -y ros-dev-tools


# Initialize rosdep
RUN apt-get update && apt-get install -y python3-rosdep && \
    rosdep init && \
    rosdep update

# Set up entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
