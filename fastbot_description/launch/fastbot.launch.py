import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_file = LaunchConfiguration("robot_file").perform(context)

    package_description = "fastbot_description"

    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "models/urdf/", robot_file
    )
    
    # Load XACRO file
    robot_desc = xacro.process_file(
        robot_desc_path, mappings={"robot_name": robot_name}
    )
    xml = robot_desc.toxml()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"{robot_name}_robot_state_publisher",
        emulate_tty=True,
        parameters=[{"use_sim_time": True, "robot_description": xml}],
        remappings=[("/robot_description", "robot_description")],  # Ensure correct topic
        output="screen",
    )

    # Joint State Publisher Node with a 2-second delay
    # joint_state_publisher_node = TimerAction(
    #     period=2.0,  # Increase delay to ensure robot_state_publisher initializes
    #     actions=[
    #         Node(
    #             package="joint_state_publisher",
    #             executable="joint_state_publisher",
    #             name=f"{robot_name}_joint_state_publisher",
    #             parameters=[{"use_sim_time": True}],
    #             output="screen",
    #         )
    #     ],
    # )

    # return [robot_state_publisher_node, joint_state_publisher_node]
    return [robot_state_publisher_node]

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="fastbot")
    robot_file_arg = DeclareLaunchArgument("robot_file", default_value="fastbot_sim.xacro")

    return LaunchDescription(
        [robot_name_arg, robot_file_arg, OpaqueFunction(function=launch_setup)]
    )
