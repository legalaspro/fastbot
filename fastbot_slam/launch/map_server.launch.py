"""
Map Server + RViz2  (single argument: map_file)

Usage:
  ros2 launch fastbot_slam map_server.launch.py map_file:=room_gazebo.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # --- Declare Launch Arguments ---
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="room_gazebo.yaml",
        description="Map YAML filename located in the package's maps/ folder"
    )
    map_file = LaunchConfiguration('map_file')

    # --- Dynamic Paths ---
    pkg_share   = FindPackageShare("fastbot_slam")
    map_yaml    = PathJoinSubstitution([pkg_share, "maps", map_file ])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "map_display.rviz"])


    return LaunchDescription([
        map_file_arg,
        
        # Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                      {'yaml_filename': map_yaml}]
        ),
        # Lifecycle Manager Node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),
        # Delay RViz by 3 seconds 
        TimerAction(
            period=3.0,
            actions=[
                 Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    name='rviz_node',
                    parameters=[{'use_sim_time': True}],
                    arguments=['-d', rviz_config]
                )
            ]
        )
    ])