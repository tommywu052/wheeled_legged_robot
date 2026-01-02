#!/usr/bin/env python3
"""
People Follower with RealSense Camera Launch File
同時啟動RealSense相機和人體追蹤功能
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 獲取套件路徑
    pkg_dir = get_package_share_directory('people_follower')
    config_file = os.path.join(pkg_dir, 'config', 'people_follower_params.yaml')
    
    # RealSense配置檔路徑
    realsense_config = '/home/robotester1/legged_robot/realsense/realsense_params.yaml'
    
    # 宣告參數
    enable_follower_arg = DeclareLaunchArgument(
        'enable_follower',
        default_value='true',
        description='啟用機器人追蹤'
    )
    
    # RealSense相機節點
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'config_file': realsense_config,
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
        }.items()
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
            }
        ]
    )
    
    return LaunchDescription([
        enable_follower_arg,
        realsense_camera,
        people_follower_node,
    ])
