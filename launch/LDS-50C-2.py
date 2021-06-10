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
    share_dir = get_package_share_directory('bluesea2')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'bluesea_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'LDS-50C-2.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode( name='bluesea_node', namespace='/', package='bluesea2', executable='bluesea_node', output='screen', emulate_tty=True, parameters=[parameter_file],)

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )

    return LaunchDescription([
        params_declare,
        driver_node,
        #tf2_node,
    ])
