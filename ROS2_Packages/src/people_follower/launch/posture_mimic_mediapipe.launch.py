#!/usr/bin/env python3
"""
姿態模仿節點啟動文件 - MediaPipe 版本
v1.5

使用方法：
ros2 launch people_follower posture_mimic_mediapipe.launch.py

作者：AI Assistant
日期：2026-01-02
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 獲取配置文件路徑
    package_dir = get_package_share_directory('people_follower')
    config_file = os.path.join(package_dir, 'config', 'posture_mimic_mediapipe_params.yaml')
    
    return LaunchDescription([
        # 聲明啟動參數
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),
        
        # 姿態模仿節點（MediaPipe 版本）
        Node(
            package='people_follower',
            executable='posture_mimic_mediapipe_node',
            name='posture_mimic_mediapipe_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            emulate_tty=True,
        ),
    ])

