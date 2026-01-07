FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    # Development tools
    python3-colcon-common-extensions \
    build-essential \
    ros-humble-xacro \
    # Serial communication
    python3-serial \
    # DDS for multi-container communication
    ros-humble-rmw-cyclonedds-cpp \
    # TF2 for transform broadcasting
    ros-humble-tf2-ros \
    # Lidar dependencies
    libboost-all-dev \
    libpcl-dev \
    libpcap-dev \
    ros-humble-pcl-conversions \
    ros-humble-diagnostic-updater \
    # Camera dependencies
    ros-humble-v4l2-camera \
    ros-humble-image-transport-plugins \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*


# ROS2 DDS Configuration
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=1

# Create ROS 2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/

# Copy package files
COPY ./fastbot_bringup /ros2_ws/src/fastbot_bringup
COPY ./fastbot_description /ros2_ws/src/fastbot_description
# COPY Motor and encoder drivers
COPY ./serial_motor /ros2_ws/src/serial_motor
COPY ./serial_motor_msgs /ros2_ws/src/serial_motor_msgs 
# COPY LIDAR drivers
COPY ./Lslidar_ROS2_driver/ /ros2_ws/src/lslidar_driver_n10


# Build workspace with colcon (symlink-install for mesh/resource file access)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install"

# Source workspace on container start (for interactive shells)
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Entrypoint
COPY ./docker/real/scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]