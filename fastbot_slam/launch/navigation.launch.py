"""
Navigation (Path Planner + Localization)

This launch file starts:
- map_server, amcl
- Nav2 planner/controller/behaviors/bt_navigator
- lifecycle manager
- RViz + global localization trigger AFTER lifecycle nodes are active

Usage:
  ros2 launch fastbot_slam navigation.launch.py map_file:=room_gazebo.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():

    # --- Launch Arguments ---
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_out = LaunchConfiguration("cmd_vel_out")
    rviz = LaunchConfiguration('rviz')

    # --- Declare Launch Arguments ---
    declare_map_arg = DeclareLaunchArgument(
        'map_file',
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

    declare_cmd_vel = DeclareLaunchArgument("cmd_vel_out", default_value="/fastbot/cmd_vel")


    # --- Dynamic paths selection ---
    pkg_share = FindPackageShare("fastbot_slam")
    config_dir = PathJoinSubstitution([pkg_share, 'config'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'pathplanning.rviz'])

    # Select configs
    map_yaml = PathJoinSubstitution([pkg_share, 'maps', map_file])
    amcl_yaml = PathJoinSubstitution([config_dir, 'amcl_config.yaml'])
    controller_yaml = PathJoinSubstitution([config_dir, 'controller.yaml'])
    bt_navigator_yaml = PathJoinSubstitution([config_dir, 'bt_navigator.yaml'])
    planner_yaml = PathJoinSubstitution([config_dir, 'planner_server.yaml'])
    behavior_yaml = PathJoinSubstitution([config_dir, 'behavior.yaml'])
    behavior_bt_xml = PathJoinSubstitution([config_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'])


    # --- Nav2 lifecycle nodes (so we can react to ACTIVE) ---
    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="", 
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )

    amcl = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace="",
        output="screen",
        parameters=[
            amcl_yaml,
            {"use_sim_time": use_sim_time},
            {'scan_topic': '/fastbot/scan'}
        ],
        remappings=[
            ('/odom', '/fastbot/odom'), 
        ]
    )

    controller_server = LifecycleNode(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace="", 
        output="screen",
        parameters=[controller_yaml, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel", cmd_vel_out),
            ('/odom', '/fastbot/odom')
        ],
    )

    planner_server = LifecycleNode(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace="", 
        output="screen",
        parameters=[planner_yaml, {"use_sim_time": use_sim_time}],
        remappings=[
                # ('/scan', '/fastbot/scan')
        ]
    )

    behavior_server = LifecycleNode(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace="",
        output="screen",
        parameters=[behavior_yaml, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel", cmd_vel_out),
            ('/odom', '/fastbot/odom')
        ],
    )

    bt_navigator = LifecycleNode(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace="", 
        output="screen",
        parameters=[
            bt_navigator_yaml,
            {"use_sim_time": use_sim_time},
            {"default_nav_to_pose_bt_xml": behavior_bt_xml},
        ],
        remappings=[
            ('/odom', '/fastbot/odom')
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        namespace="", 
        output="screen",
        parameters=[
            {
                "autostart": True,
                "use_sim_time": use_sim_time,
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }
        ],
    )

    # --- Things we want to start *after* lifecycle is ACTIVE ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
    )

    global_loc_trigger = Node(
        package="fastbot_slam",
        executable="trigger_global_localization.py",
        name="global_loc_trigger",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {
                "vel_topic": cmd_vel_out,
                "spin_duration": 10.0,
                "angular_speed": 0.4,
            },
        ],
    )

    # Start global localization when AMCL is ACTIVE
    start_global_loc_when_amcl_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=amcl,
            goal_state="active",
            entities=[global_loc_trigger],
        )
    )

    # Start RViz when BT navigator is ACTIVE (Nav2 fully up)
    start_rviz_when_bt_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bt_navigator,
            goal_state="active",
            entities=[rviz_node],
        )
    )

    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time,
        declare_rviz,
        declare_cmd_vel,
        LogInfo(msg=["Using sim time: ", use_sim_time]),

        # Nav2
        map_server,
        amcl,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,

        start_global_loc_when_amcl_active,
        start_rviz_when_bt_active,
    ])