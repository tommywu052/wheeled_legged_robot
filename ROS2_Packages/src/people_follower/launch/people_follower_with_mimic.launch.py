#!/usr/bin/env python3
"""
人體跟隨 + 姿態模仿 組合啟動文件

同時啟動：
1. 人體跟隨節點（使用 YOLOv8 檢測）
2. 姿態模仿節點（使用 YOLOv8-Pose 檢測姿態）

使用方法：
    # 完整功能（跟隨 + 模仿）
    ros2 launch people_follower people_follower_with_mimic.launch.py
    
    # 只跟隨，不模仿姿態
    ros2 launch people_follower people_follower_with_mimic.launch.py enable_mimic:=false
    
    # 只模仿姿態，不跟隨
    ros2 launch people_follower people_follower_with_mimic.launch.py enable_follower:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 聲明啟動參數
    enable_follower_arg = DeclareLaunchArgument(
        'enable_follower',
        default_value='true',
        description='啟用人體跟隨功能'
    )
    
    enable_mimic_arg = DeclareLaunchArgument(
        'enable_mimic',
        default_value='true',
        description='啟用姿態模仿功能'
    )
    
    # 配置文件路徑
    follower_config = PathJoinSubstitution([
        FindPackageShare('people_follower'),
        'config',
        'people_follower_yolo_params.yaml'
    ])
    
    mimic_config = PathJoinSubstitution([
        FindPackageShare('people_follower'),
        'config',
        'posture_mimic_params.yaml'
    ])
    
    # 人體跟隨節點（YOLOv8）
    follower_node = Node(
        package='people_follower',
        executable='people_follower_yolo_node',
        name='people_follower_yolo_node',
        output='screen',
        parameters=[
            follower_config,
            {'enable_follower': LaunchConfiguration('enable_follower')}
        ]
    )
    
    # 姿態模仿節點（YOLOv8-Pose）
    mimic_node = Node(
        package='people_follower',
        executable='posture_mimic_node',
        name='posture_mimic_node',
        output='screen',
        parameters=[
            mimic_config,
            {'enable_mimic': LaunchConfiguration('enable_mimic')}
        ]
    )
    
    return LaunchDescription([
        enable_follower_arg,
        enable_mimic_arg,
        follower_node,
        mimic_node
    ])


