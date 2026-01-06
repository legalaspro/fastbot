# =============================================================================
# Fastbot Dev Container - Full development environment
#
# Single container with ROS2, Gazebo, SLAM, Nav2, and WebApp tools
# =============================================================================
FROM osrf/ros:humble-desktop

# Install development dependencies
RUN apt-get update && apt-get install -y \
    # Development tools
    git \
    vim \
    wget \
    curl \
    build-essential \
    gdb \
    # DDS for multi-container communication
    ros-humble-rmw-cyclonedds-cpp \
    # Gazebo Classic
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    # Robot Control
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # Robot Description
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    # Transforms
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    # Navigation/Localization
    ros-humble-robot-localization \
    # SLAM packages
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    # Navigation2
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # Teleop
    ros-humble-teleop-twist-keyboard \
    # RQt tools
    ros-humble-rqt-graph \
    ros-humble-rqt-tf-tree \
    ros-humble-rqt-topic \
    ros-humble-rqt-image-view \
    # WebApp dependencies
    ros-humble-rosbridge-suite \
    ros-humble-web-video-server \
    # TF2 dependencies for tf2_web_republisher_py
    python3-numpy \
    python3-pip \
    # Node.js for webapp development
    nodejs \
    npm \
    && rm -rf /var/lib/apt/lists/* \
    # Python quaternion library (used by tf2_web_republisher)
    && pip3 install pyquaternion 

# ROS2 DDS Configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=1

# Create ROS 2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/

# Disable core dumps (prevents large files on crash)
RUN echo "ulimit -c 0" >> ~/.bashrc

# Source ROS2 and workspace on shell start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Ensure XDG_RUNTIME_DIR exists" >> ~/.bashrc && \
    echo "mkdir -p /tmp/runtime-root 2>/dev/null" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Helpful aliases" >> ~/.bashrc && \
    echo "alias cb='cd /ros2_ws && colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias cbs='cd /ros2_ws && colcon build --symlink-install --packages-select'" >> ~/.bashrc && \
    echo "alias rt='ros2 topic list'" >> ~/.bashrc && \
    echo "alias rn='ros2 node list'" >> ~/.bashrc && \
    echo "alias webapp='cd /ros2_ws/src/fastbot_webapp && npm run build'" >> ~/.bashrc

CMD ["bash"]

