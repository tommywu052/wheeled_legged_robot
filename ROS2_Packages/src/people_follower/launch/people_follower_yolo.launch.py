#!/usr/bin/env python3
"""
People Follower YOLO Launch File
啟動基於YOLOv8的人體追蹤功能
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 獲取套件路徑
    pkg_dir = get_package_share_directory('people_follower')
    config_file = os.path.join(pkg_dir, 'config', 'people_follower_yolo_params.yaml')
    
    # 宣告參數
    enable_follower_arg = DeclareLaunchArgument(
        'enable_follower',
        default_value='true',
        description='啟用機器人追蹤'
    )
    
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8n.pt',
        description='YOLO模型: yolov8n.pt (快) / yolov8s.pt (準確)'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='YOLO信心閾值 (0.0-1.0)'
    )
    
    # People Follower YOLO節點
    people_follower_node = Node(
        package='people_follower',
        executable='people_follower_yolo_node',
        name='people_follower_yolo',
        output='screen',
        parameters=[
            config_file,
            {
                'enable_follower': LaunchConfiguration('enable_follower'),
                'yolo_model': LaunchConfiguration('yolo_model'),
                'confidence_threshold': LaunchConfiguration('confidence'),
            }
        ]
    )
    
    return LaunchDescription([
        enable_follower_arg,
        yolo_model_arg,
        confidence_arg,
        people_follower_node,
    ])

