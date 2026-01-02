#!/usr/bin/env python3
"""
People Follower Launch File
啟動人體追蹤功能（不包含RealSense，假設相機已單獨啟動）
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
    config_file = os.path.join(pkg_dir, 'config', 'people_follower_params.yaml')
    
    # 宣告參數
    enable_follower_arg = DeclareLaunchArgument(
        'enable_follower',
        default_value='true',
        description='啟用機器人追蹤 (false = 僅偵測)'
    )
    
    publish_debug_arg = DeclareLaunchArgument(
        'publish_debug',
        default_value='true',
        description='發佈除錯視覺化影像'
    )
    
    detection_method_arg = DeclareLaunchArgument(
        'detection_method',
        default_value='hog',
        description='偵測方法: hog 或 dnn'
    )
    
    # People Follower節點
    people_follower_node = Node(
        package='people_follower',
        executable='people_follower_node',
        name='people_follower_node',
        output='screen',
        parameters=[
            config_file,
            {
                'enable_follower': LaunchConfiguration('enable_follower'),
                'publish_debug_image': LaunchConfiguration('publish_debug'),
                'detection_method': LaunchConfiguration('detection_method'),
            }
        ],
        remappings=[
            # RealSense話題重新對應（如果需要）
            # 預設話題：/camera/camera/color/image_raw
            # 如需修改，取消下面的註解：
            # ('/camera/camera/color/image_raw', '/your_camera/color/image_raw'),
        ]
    )
    
    return LaunchDescription([
        enable_follower_arg,
        publish_debug_arg,
        detection_method_arg,
        people_follower_node,
    ])
