#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os



def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('bluesea2')     
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros,"launch"),"LDS-E300-E.launch")
    )
    return LaunchDescription([
        gazebo_launch
        
    ])