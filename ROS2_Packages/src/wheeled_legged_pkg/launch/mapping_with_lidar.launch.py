#!/usr/bin/env python3
"""
轮足机器人 + N10 Lidar 建图 Launch 文件
功能：
1. 启动机器人基础节点 (wl_base_node)
2. 发布机器人URDF模型 (robot_state_publisher) - 自动发布所有TF变换
3. 启动N10 Lidar驱动
4. 启动SLAM Toolbox进行建图
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 获取URDF文件路径
    urdf_pkg_path = get_package_share_directory('wheel_legged_urdf_pkg')
    urdf_file = os.path.join(urdf_pkg_path, 'urdf', 'wheel_legged_urdf_pkg.urdf')
    
    # 读取URDF文件
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()


    # 临时方案：使用静态TF替代（仅用于无机器人情况下测试URDF可视化）
    # 注意：这不能用于实际建图！只能用于查看模型
    # static_odom_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_base_link_publisher',
    #     arguments=['0', '0', '0.25', '0', '0', '0', 'odom', 'base_link'],
    #     output='screen'
    # )
    
    # 1. Robot State Publisher - 发布URDF中定义的所有TF变换
    # 这会自动发布 base_link -> laser 的变换（从URDF中读取）
    # URDF中定义的Lidar位置：
    #   X = 0.0451m (向前4.5cm)
    #   Y = -0.0007m (基本在中心)
    #   Z = 0.1038m (向上10.4cm，相对于base_link)
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

    # 2b. Joint State Publisher - 发布关节状态
    # 为URDF中的revolute关节提供默认角度，消除RViz中的"No transform"错误
    # 这会发布 /joint_states 话题，robot_state_publisher会读取它来计算revolute关节的TF
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
    
    # 2. 启动N10 Lidar驱动
    # 注意：确保lidar话题名称为 /scan
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
    #         # 基本参数
    #         'lidar_type': 'X10',
    #         'lidar_model': 'N10',
    #         'serial_port': '/dev/ttyUSB0',
    #         'device_ip': '192.168.1.200',
    #         'msop_port': 2368,
    #         'difop_port': 2369,
    #         'packet_rate': 188.0,
            
    #         # 扫描参数
    #         'frame_id': 'laser',  # 使用URDF中定义的frame
    #         'pointcloud_topic': 'lslidar_point_cloud',
    #         'laserscan_topic': 'scan',
    #         'use_time_service': False,
    #         'use_first_point_time': False,
    #         'publish_scan': True,
    #         'use_high_precision': True,
    #         'publish_multiecholaserscan': False,
    #         'enable_noise_filter': False,
    #         'N10Plus_hz': 10,
            
    #         # 范围和角度参数
    #         'min_range': 0.15,
    #         'max_range': 50.0,
    #         'angle_disable_min': 0,
    #         'angle_disable_max': 0,
            
    #         # ⭐ 关键：修正左右镜像问题
    #         # 启用坐标变换并旋转180°（π弧度）
    #         'is_pretreatment': True,      # 启用坐标变换预处理
    #         'x_offset': 0.0,
    #         'y_offset': 0.0,
    #         'z_offset': 0.0,
    #         'roll': 0.0,
    #         'pitch': 0.0,
    #         'yaw': 3.14159265359,         # 180° (π) - 修正镜像
            
    #         # 高级变换（如果simple参数不生效，尝试使用矩阵变换）
    #         'is_MatrixTransformation': False,
    #     }],
    #     remappings=[
    #         # 将/x10/scan映射到/scan，供SLAM使用
    #         ('/x10/scan', '/scan'),
    #     ]
    # )
    # 3. 启动SLAM Toolbox进行异步建图
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

        # 5. 启动RViz进行可视化
    # 使用默认配置启动RViz（用户需要手动添加显示项）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    return LaunchDescription([
        #static_odom_to_base,  # 临时静态TF（仅用于测试，不能实际建图）
        robot_state_publisher_node,
        #joint_state_publisher_node,  # 发布关节状态，修复RViz中的"No transform"错误
        #lidar_node,
        slam_node,
        rviz_node,  # RViz可视化
    ])

