import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 获取功能包共享目录路径
    pkg_share = get_package_share_directory('wheel_legged_urdf_pkg')
    
    # 定义默认的URDF模型路径和RViz配置路径
    default_model_path = os.path.join(pkg_share, 'urdf', 'wheel_legged_urdf_pkg.urdf')
    
    # 检查是否有config目录，如果有则使用其中的rviz配置
    config_dir = os.path.join(pkg_share, 'config')
    if os.path.exists(config_dir):
        default_rviz_config_path = os.path.join(config_dir, 'display.rviz')
    else:
        # 如果没有config目录，则使用默认路径
        default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # 声明启动参数 //这是 ROS2 中一个常用的官方节点，功能是带 GUI 界面的关节状态发布器
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui' 
    )
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 新增：声明URDF发布频率参数
    publish_frequency_arg = DeclareLaunchArgument(
        name='publish_frequency',
        default_value='30.0',
        description='Frequency (in Hz) for publishing URDF transforms'
    )

    # 处理xacro文件（如果模型是xacro格式）
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 启动 robot_state_publisher 节点，添加发布频率参数
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': LaunchConfiguration('publish_frequency')  # 新增发布频率参数
        }]
    )

    # 根据gui参数条件启动 joint_state_publisher 或 joint_state_publisher_gui
    # 同时添加发布频率参数
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{
            'rate': LaunchConfiguration('publish_frequency'),  # 使用相同的发布频率参数
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{
            'rate': LaunchConfiguration('publish_frequency'),  # 使用相同的发布频率参数
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 启动 rviz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_config_arg,
        use_sim_time_arg,
        publish_frequency_arg,  # 新增的频率参数声明
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])