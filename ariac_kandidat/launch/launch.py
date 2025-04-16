import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    # Get tutorial number, but don't use it to conditionally change behavior
    tutorial_num = int(LaunchConfiguration("tutorial").perform(context))

    # Always include MoveIt
    parameters = {
        "use_sim_time": True,
        "use_moveit": True
    }

    # Build MoveIt config
    urdf = os.path.join(
        get_package_share_directory("ariac_description"),
        "urdf/ariac_robots/ariac_robots.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("ariac_tutorials"),
                "config/moveit_config.yaml"
            )
        )
        .to_moveit_configs()
    )

    parameters.update(moveit_config.to_dict())

    # Launch tutorial node
    tutorial_node = Node(
        package="ariac_kandidat",
        executable="agv_test.py",
        output="screen",
        parameters=[parameters]
    )

    # Always include Move Group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ariac_moveit_config"),
            "/launch/ariac_robots_moveit.launch.py"
        ])
    )

    return [tutorial_node, move_group]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("tutorial", default_value="1", description="tutorial number to run (1–8)"),
        OpaqueFunction(function=launch_setup)
    ])
