"""
Map Server + RViz2 (single argument: map_file)

Usage:
  ros2 launch fastbot_slam map_server.launch.py map_file:=room_gazebo.yaml
  ros2 launch fastbot_slam map_server.launch.py use_sim_time:=False rviz:=False  # Real robot
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Launch Arguments ---
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # --- Declare Launch Arguments ---
    declare_map_file = DeclareLaunchArgument(
        "map_file",
        default_value="room_gazebo.yaml",
        description="Map YAML filename located in the package's maps/ folder"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz (set False for headless robot)'
    )

    # --- Dynamic Paths ---
    pkg_share = FindPackageShare("fastbot_slam")
    map_yaml = PathJoinSubstitution([pkg_share, "maps", map_file])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "map_display.rviz"])

    return LaunchDescription([
        declare_map_file,
        declare_use_sim_time,
        declare_rviz,

        # Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_yaml}
            ]
        ),
        # Lifecycle Manager Node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
        # RViz (conditional, delayed)
        TimerAction(
            period=3.0,
            condition=IfCondition(rviz),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_node',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-d', rviz_config]
                )
            ]
        )
    ])