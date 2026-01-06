"""
Path Planner + Localization launch for fastbot.

This launch file starts:
- map_server (static map)
- amcl (localization)
- Nav2 planner, controller, behaviors, bt_navigator
- lifecycle manager
- RViz (delayed)

Usage:
  ros2 launch fastbot_slam pathplanner.launch.py map_file:="room_gazebo.yaml"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- Launch Arguments ---
    map_file = LaunchConfiguration('map_file')
    declare_map_arg = DeclareLaunchArgument(
        'map_file',
        default_value="room_gazebo.yaml",
        description="Map YAML filename located in the package's maps/ folder"
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Dynamic paths selection ---
    pkg_share = FindPackageShare("fastbot_slam")
    config_dir = PathJoinSubstitution([pkg_share, 'config'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'pathplanning.rviz'])

    # Select configs
    controller_yaml = PathJoinSubstitution([config_dir, 'controller.yaml'])
    bt_navigator_yaml = PathJoinSubstitution([config_dir, 'bt_navigator.yaml'])
    planner_yaml = PathJoinSubstitution([config_dir, 'planner_server.yaml'])
    behavior_yaml = PathJoinSubstitution([config_dir, 'behavior.yaml'])
    behavior_bt_xml = PathJoinSubstitution([config_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'])

    # --- Include Localization (map_server + amcl) ---
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_share, '/launch/localization.launch.py'
        ]),
        launch_arguments={
            'map_file': map_file,
            'use_sim_time': use_sim_time,
            'rviz': 'False',
        }.items()
    )

    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time,   

        LogInfo(msg=["Using sim time: ", use_sim_time]),
        
        # Localization first (required!)
        localization,
    
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[
                ('/cmd_vel', '/fastbot/cmd_vel'),
                ('/odom', '/fastbot/odom')
            ]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml],
            remappings=[
                # ('/scan', '/fastbot/scan')
            ]
        ),  
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[behavior_yaml],
            remappings=[
                ('/cmd_vel', '/fastbot/cmd_vel'),
                ('/odom', '/fastbot/odom')
            ],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_navigator_yaml, 
                { 'default_nav_to_pose_bt_xml' : behavior_bt_xml}
            ],
            remappings=[
                ('/odom', '/fastbot/odom'),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True,
                        'use_sim_time': use_sim_time,
                        'node_names': [
                            'planner_server',
                            'controller_server',
                            'behavior_server',
                            'bt_navigator'
                        ]}
            ]
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=[
                        "-d",
                        rviz_config,
                    ],
                ),
            ]
        )
    ])