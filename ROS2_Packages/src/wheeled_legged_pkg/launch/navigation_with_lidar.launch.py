#!/usr/bin/env python3
"""
輪足機器人 + N10 Lidar 完整導航 Launch 文件
功能：
1. 加載地圖
2. 啟動AMCL定位
3. 啟動完整Nav2導航棧（路徑規劃、控制、代價地圖等）
4. 啟動機器人狀態發布
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    
    # 獲取包路徑
    pkg_dir = os.path.expanduser('~/legged_robot/ROS2_Packages/src/wheeled_legged_pkg')
    
    # 獲取URDF文件路徑
    urdf_file = os.path.expanduser('~/legged_robot/ROS2_Packages/src/wheel_legged_urdf_pkg/urdf/wheel_legged_urdf_pkg.urdf')
    
    # 讀取URDF文件
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # 地圖文件參數
    default_map_path = os.path.expanduser('~/maps/my_map2.yaml')
    
    # Nav2參數文件
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # 聲明啟動參數
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file'
    )
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to Nav2 params file'
    )
    
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    # 打印路徑用於調試
    print(f"[INFO] Using map file: {default_map_path}")
    print(f"[INFO] Using params file: {nav2_params_file}")
    print(f"[INFO] Map file exists: {os.path.exists(default_map_path)}")
    print(f"[INFO] Params file exists: {os.path.exists(nav2_params_file)}")
    
    # 1. Robot State Publisher
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
    
    # 2. Map Server（加載地圖）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml_file}
        ]
    )
    
    # 3. AMCL（定位）
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )
    
    # 4. Controller Server（路徑跟踪控制器）
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )
    
    # 5. Planner Server（全局路徑規劃器）
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )
    
    # 6. Behavior Server（恢復行為）
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )
    
    # 7. BT Navigator（行為樹導航器）
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )
    
    # 8. Waypoint Follower（路點跟隨器）
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )
    
    # 9. Velocity Smoother（速度平滑器）
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel')
        ]
    )
    
    # 10. Lifecycle Manager - Localization（定位生命週期管理）
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # 11. Lifecycle Manager - Navigation（導航生命週期管理）
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )
    
    # 12. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    return LaunchDescription([
        declare_map_arg,
        declare_params_file_arg,
        robot_state_publisher_node,
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
        rviz_node,
    ])
