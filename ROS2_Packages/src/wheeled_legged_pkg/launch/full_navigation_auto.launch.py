#!/usr/bin/env python3
"""
完整導航系統自動啟動 Launch 文件
包含：機器人基礎節點、激光雷達、導航系統、自動初始化
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 獲取包路徑
    pkg_dir = get_package_share_directory('wheeled_legged_pkg')
    
    # 聲明參數
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='機器人串口設備路徑'
    )
    
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='激光雷達串口設備路徑'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'maps', 'my_map2.yaml'),
        description='地圖文件路徑'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 參數文件路徑'
    )
    
    declare_init_x = DeclareLaunchArgument(
        'init_x',
        default_value='0.0',
        description='AMCL 初始 X 坐標（地圖坐標系）'
    )
    
    declare_init_y = DeclareLaunchArgument(
        'init_y',
        default_value='0.0',
        description='AMCL 初始 Y 坐標（地圖坐標系）'
    )
    
    declare_init_yaw = DeclareLaunchArgument(
        'init_yaw',
        default_value='0.0',
        description='AMCL 初始朝向（弧度）'
    )
    
    declare_wait_time = DeclareLaunchArgument(
        'wait_time',
        default_value='15.0',
        description='啟動後等待多少秒再初始化 AMCL'
    )
    
    # 獲取參數
    serial_port = LaunchConfiguration('serial_port')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')
    wait_time = LaunchConfiguration('wait_time')
    
    # ========== 1. 機器人基礎節點（自動選擇串口）==========
    wl_base_node = Node(
        package='wheeled_legged_pkg',
        executable='wl_base_node',
        name='wl_base_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'auto_select': True  # 自動選擇串口
        }]
    )
    
    # ========== 2. 激光雷達節點 ==========
    # 注意：這裡假設 lslidar_driver 包在同一個工作空間或已經 source
    # 如果在不同工作空間，需要提前 source 或使用絕對路徑
    
    # 嘗試獲取 lslidar_driver 包路徑
    try:
        lslidar_pkg_dir = get_package_share_directory('lslidar_driver')
        lslidar_launch_file = os.path.join(lslidar_pkg_dir, 'launch', 'lsn10_launch.py')
        
        lidar_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lslidar_launch_file),
            launch_arguments={}.items()
        )
    except Exception as e:
        print(f"⚠️  警告：無法找到 lslidar_driver 包: {e}")
        print("   請手動啟動激光雷達或確保已 source 激光雷達工作空間")
        lidar_node = ExecuteProcess(
            cmd=['echo', '⚠️  激光雷達節點未啟動，請手動啟動：ros2 launch lslidar_driver lsn10_launch.py'],
            output='screen'
        )
    
    # ========== 3. 導航系統（包含所有 Nav2 組件）==========
    navigation_launch_file = os.path.join(pkg_dir, 'launch', 'navigation_with_lidar.launch.py')
    
    navigation_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'map': map_file,
            'params_file': params_file
        }.items()
    )
    
    # ========== 4. 自動初始化 AMCL（延遲啟動）==========
    auto_init_node = Node(
        package='wheeled_legged_pkg',
        executable='auto_init_amcl.py',
        name='auto_init_amcl',
        output='screen',
        parameters=[{
            'wait_time': wait_time,
            'init_x': init_x,
            'init_y': init_y,
            'init_yaw': init_yaw
        }]
    )
    
    # 使用 TimerAction 延遲啟動自動初始化節點
    delayed_auto_init = TimerAction(
        period=5.0,  # 5秒後啟動
        actions=[auto_init_node]
    )
    
    # ========== 創建 Launch Description ==========
    ld = LaunchDescription()
    
    # 添加參數聲明
    ld.add_action(declare_serial_port)
    ld.add_action(declare_lidar_port)
    ld.add_action(declare_map_file)
    ld.add_action(declare_params_file)
    ld.add_action(declare_init_x)
    ld.add_action(declare_init_y)
    ld.add_action(declare_init_yaw)
    ld.add_action(declare_wait_time)
    
    # 按順序添加節點
    ld.add_action(wl_base_node)           # 1. 機器人基礎
    ld.add_action(lidar_node)              # 2. 激光雷達
    ld.add_action(navigation_system)       # 3. 導航系統
    ld.add_action(delayed_auto_init)       # 4. 自動初始化（延遲）
    
    return ld

