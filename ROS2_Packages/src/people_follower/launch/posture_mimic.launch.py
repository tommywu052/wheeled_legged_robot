#!/usr/bin/env python3
"""
姿態模仿啟動文件

單獨啟動姿態模仿功能（不跟隨，只模仿姿態）

使用方法：
    ros2 launch people_follower posture_mimic.launch.py
    ros2 launch people_follower posture_mimic.launch.py enable_mimic:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 聲明啟動參數
    enable_mimic_arg = DeclareLaunchArgument(
        'enable_mimic',
        default_value='true',
        description='啟用姿態模仿功能'
    )
    
    # 配置文件路徑
    config_file = PathJoinSubstitution([
        FindPackageShare('people_follower'),
        'config',
        'posture_mimic_params.yaml'
    ])
    
    # 姿態模仿節點
    posture_mimic_node = Node(
        package='people_follower',
        executable='posture_mimic_node',
        name='posture_mimic_node',
        output='screen',
        parameters=[
            config_file,
            {'enable_mimic': LaunchConfiguration('enable_mimic')}
        ],
        remappings=[
            # 如果需要重映射話題，在這裡添加
        ]
    )
    
    return LaunchDescription([
        enable_mimic_arg,
        posture_mimic_node
    ])


