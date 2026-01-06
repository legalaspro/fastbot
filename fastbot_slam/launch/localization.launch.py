"""
Localization Server + RViz2 (single argument: map_file)

Usage:
  ros2 launch localization_server localization.launch.py map_file:=room_gazebo.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
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
    declare_map_arg = DeclareLaunchArgument(
        "map_file",
        default_value="room_gazebo.yaml",
        description="Map YAML filename located in the package's maps/ folder"
    )
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',  # Default: start RViz
        description='Start RViz if true'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Dynamic Paths ---
    pkg_share = FindPackageShare("fastbot_slam")
    map_yaml = PathJoinSubstitution([
        pkg_share,
        "maps", 
        map_file
    ])
    amcl_yaml = PathJoinSubstitution([
        pkg_share, 
        "config", 
        'amcl_config.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        pkg_share,
        "rviz",
        "localizer.rviz"
    ])

    # --- Logs ---
    log_map = LogInfo(msg=["Map YAML: ", map_yaml])
    
    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time,
        declare_rviz_arg,

        log_map,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_yaml}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {"use_sim_time": use_sim_time},
                amcl_yaml,
                {'scan_topic': '/fastbot/scan'}
            ],
            remappings=[
                ('/odom', '/fastbot/odom'), 
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': 
                            [
                                'map_server', 
                                'amcl'
                            ]
                        }]
        ),
        TimerAction(
            period=3.0,
            condition=IfCondition(rviz),
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                    arguments=[
                        "-d",
                        rviz_config,
                    ],
                ),
            ]
        )
    ])