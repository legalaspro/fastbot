"""
Cartographer SLAM launch.

Usage:
  ros2 launch fastbot_slam cartographer.launch.py
  ros2 launch fastbot_slam cartographer.launch.py use_sim_time:=False rviz:=False  # Real robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time (True for Gazebo, False for real robot)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz (set False for headless robot)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # --- Dynamic paths ---
    pkg_share = FindPackageShare("fastbot_slam")
    cartographer_config_dir = PathJoinSubstitution([pkg_share, 'config'])
    configuration_basename = "cartographer.lua"
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", 'mapping.rviz'])

    # ---- Logs ---
    log_time_mode = LogInfo(msg=["use_sim_time = ", use_sim_time])
    log_rviz_mode = LogInfo(msg=["rviz = ", rviz])

    # --- Nodes ---
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/scan', '/fastbot/scan'),
            ('/odom', '/fastbot/odom'),
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # RViz2 Node (conditional, with delay)
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz_node',
                output='screen',
                parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
                arguments=['-d', rviz_config],
                condition=IfCondition(rviz)
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        log_time_mode,
        log_rviz_mode,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])