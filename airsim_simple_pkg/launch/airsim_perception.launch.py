#!/usr/bin/env python3

"""
Launch file for AirSim Perception Node

This launch file starts the AirSim perception node with the specified parameters.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for AirSim perception node"""
    
    # Get the package directory
    pkg_dir = get_package_share_directory('airsim_simple_pkg')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'vis_local_pcl.rviz')
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz for visualization'
    )
    
    # Create the node
    airsim_perception_node = Node(
        package='airsim_simple_pkg',
        executable='airsim_perception_node',
        name='airsim_perception_node',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # You can add topic remappings here if needed
            # ('/airsim/odom', '/robot/odom'),
            # ('/airsim/pointcloud', '/robot/pointcloud'),
        ]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        log_level_arg,
        use_rviz_arg,
        airsim_perception_node,
        rviz_node
    ])
