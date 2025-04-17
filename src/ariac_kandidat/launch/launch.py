import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, AllSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch the competition script node (competition_script.py)
        ExecuteProcess(
            cmd=['ros2', 'run', 'ariac_kandidat', 'competition_script.py'],
            output='screen',
            name='competition_script',
        ),

        # Launch the CCS node (ccs_node.py)
        ExecuteProcess(
            cmd=['ros2', 'run', 'ariac_kandidat', 'ccs_node.py'],
            output='screen',
            name='ccs_node',
        )
    ])