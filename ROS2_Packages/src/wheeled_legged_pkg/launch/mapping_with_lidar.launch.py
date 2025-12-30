#!/usr/bin/env python3
"""
輪足機器人 + N10 Lidar 建圖 Launch 文件
功能：
1. 啟動機器人基礎節點 (wl_base_node)
2. 發布機器人URDF模型 (robot_state_publisher) - 自動發布所有TF變換
3. 啟動N10 Lidar驅動
4. 啟動SLAM Toolbox進行建圖
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 獲取URDF文件路徑
    urdf_pkg_path = get_package_share_directory('wheel_legged_urdf_pkg')
    urdf_file = os.path.join(urdf_pkg_path, 'urdf', 'wheel_legged_urdf_pkg.urdf')
    
    # 讀取URDF文件
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()


    # 臨時方案：使用靜態TF替代（僅用於無機器人情況下測試URDF可視化）
    # 注意：這不能用於實際建圖！只能用於查看模型
    # static_odom_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_base_link_publisher',
    #     arguments=['0', '0', '0.25', '0', '0', '0', 'odom', 'base_link'],
    #     output='screen'
    # )
    
    # 1. Robot State Publisher - 發布URDF中定義的所有TF變換
    # 這會自動發布 base_link -> laser 的變換（從URDF中讀取）
    # URDF中定義的Lidar位置：
    #   X = 0.0451m (向前4.5cm)
    #   Y = -0.0007m (基本在中心)
    #   Z = 0.1038m (向上10.4cm，相對於base_link)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # 2b. Joint State Publisher - 發布關節狀態
    # 為URDF中的revolute關節提供默認角度，消除RViz中的"No transform"錯誤
    # 這會發布 /joint_states 話題，robot_state_publisher會讀取它來計算revolute關節的TF
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': robot_description,
    #         'use_sim_time': False
    #     }]
    # )
    
    # 2. 啟動N10 Lidar驅動
    # 注意：確保lidar話題名稱為 /scan
    # lidar_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('lslidar_driver'),
    #         '/launch/lslidar_x10_launch.py'
    #     ])
    # )
    # lidar_node = Node(
    #     package='lslidar_driver',
    #     executable='lslidar_driver_node',
    #     name='lslidar_driver_node',
    #     namespace='x10',
    #     output='screen',
    #     parameters=[{
    #         # 基本參數
    #         'lidar_type': 'X10',
    #         'lidar_model': 'N10',
    #         'serial_port': '/dev/ttyUSB0',
    #         'device_ip': '192.168.1.200',
    #         'msop_port': 2368,
    #         'difop_port': 2369,
    #         'packet_rate': 188.0,
            
    #         # 掃描參數
    #         'frame_id': 'laser',  # 使用URDF中定義的frame
    #         'pointcloud_topic': 'lslidar_point_cloud',
    #         'laserscan_topic': 'scan',
    #         'use_time_service': False,
    #         'use_first_point_time': False,
    #         'publish_scan': True,
    #         'use_high_precision': True,
    #         'publish_multiecholaserscan': False,
    #         'enable_noise_filter': False,
    #         'N10Plus_hz': 10,
            
    #         # 範圍和角度參數
    #         'min_range': 0.15,
    #         'max_range': 50.0,
    #         'angle_disable_min': 0,
    #         'angle_disable_max': 0,
            
    #         # ⭐ 關鍵：修正左右鏡像問題
    #         # 啟用坐標變換並旋轉180°（π弧度）
    #         'is_pretreatment': True,      # 啟用坐標變換預處理
    #         'x_offset': 0.0,
    #         'y_offset': 0.0,
    #         'z_offset': 0.0,
    #         'roll': 0.0,
    #         'pitch': 0.0,
    #         'yaw': 3.14159265359,         # 180° (π) - 修正鏡像
            
    #         # 高級變換（如果simple參數不生效，嘗試使用矩陣變換）
    #         'is_MatrixTransformation': False,
    #     }],
    #     remappings=[
    #         # 將/x10/scan映射到/scan，供SLAM使用
    #         ('/x10/scan', '/scan'),
    #     ]
    # )
    # 3. 啟動SLAM Toolbox進行異步建圖
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'mode': 'asynchronous',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'resolution': 0.05,
            'max_laser_range': 10.0,
            'min_laser_range': 0.21,
            'minimum_time_interval': 0.1,
            'scan_min_range_readings': 520,
            'map_update_interval': 0.5,
            'enable_loop_closure': True,
            'linear_update': 0.05,
            'angular_update': 0.05,
            # 性能优化参数
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.15,
            'minimum_travel_heading': 0.15,
            # 调试参数
            'do_loop_closing': True,
            'loop_search_maximum_distance': 3.0,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance': 0.4,
            
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,
            'debug_logging': False,
        }]
        # remappings=[
        #     ('/scan', '/x10/scan'),
        # ]
    )

        # 5. 啟動RViz進行可視化
    # 使用默認配置啟動RViz（用戶需要手動添加顯示項）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    return LaunchDescription([
        #static_odom_to_base,  # 臨時靜態TF（僅用於測試，不能實際建圖）
        robot_state_publisher_node,
        #joint_state_publisher_node,  # 發布關節狀態，修復RViz中的"No transform"錯誤
        #lidar_node,
        slam_node,
        rviz_node,  # RViz可視化
    ])

