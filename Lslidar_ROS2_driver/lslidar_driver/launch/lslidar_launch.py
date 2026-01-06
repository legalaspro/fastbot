#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_link',
        description='Frame ID for the lidar data'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for the laser scan data'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/lslidar',
        description='Serial port for the lidar'
    )

    # Get launch configurations
    frame_id = LaunchConfiguration('frame_id')
    scan_topic = LaunchConfiguration('scan_topic')
    serial_port = LaunchConfiguration('serial_port')

    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lsx10.yaml')

    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[driver_dir,
                    {'frame_id': frame_id,
                     'scan_topic': scan_topic,
                     'serial_port_': serial_port}],
    )


    rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz', 'lslidar.rviz')

    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')

    return LaunchDescription([
        frame_id_arg,
        scan_topic_arg,
        serial_port_arg,
        driver_node,
        # rviz_node,
    ])

