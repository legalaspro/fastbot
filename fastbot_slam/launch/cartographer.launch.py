"""
Cartographer + RViz2 launch (sim).

Usage:
  ros2 launch fastbot_slam cartographer.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True', 
        description='Whether simultaion or real robot used'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Dynamic paths selection ---
    pkg_share = FindPackageShare("fastbot_slam")
    cartographer_config_dir = PathJoinSubstitution([pkg_share, 'config'])
    configuration_basename = "cartographer.lua"
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", 'mapping.rviz'])
    
    # ---- Logs ---
    log_config_choice = LogInfo(
        msg=["Selected cartographer config: ", configuration_basename]
    )
    log_time_mode = LogInfo(
        msg=["use_sim_time = ", use_sim_time]
    )

    # RViz2 Node with Delay
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config]
    )

    # Delay RViz by 3 seconds 
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([ 
        use_sim_time_arg,
        log_config_choice,
        log_time_mode,
        # Node(
        #     package='fastbot_slam',
        #     executable='timestamp_filter_node.py',
        #     name='timestamp_filter',
        #     output='screen',
        #     parameters=[
        #         {
        #             'scan_input': '/fastbot_1/scan', # LaserScan input
        #             'scan_output': '/scan',
        #             'odom_input': '/fastbot_1/odom', # Odometry input 
        #             'odom_output': '/odom'
        #         }]
        # ),
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[
                ('/scan', '/fastbot/scan'), # LaserScan input
                ('/odom', '/fastbot/odom'), # nav_msgs/Odometry input 
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        delayed_rviz
    ]) 