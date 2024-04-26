from launch import LaunchDescription
from launch.actions import TimerAction, Shutdown, DeclareLaunchArgument
from launch_ros.actions import Node
from tracetools_launch.action import Trace
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import time

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='es-car',
            executable='steering',
	    name='steering',
	    output='screen'
        ),
	
	Node(
            package='es-car',
            executable='throttle',
            name='throttle',
            output='screen'
        ),
    ])
