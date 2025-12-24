#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明可配置参数
    stm32_port_arg = DeclareLaunchArgument(
        'stm32_port', default_value=TextSubstitution(text='/dev/ttyUSB1'),
        description='STM32控制器连接的串口设备'
    )
    stm32_baudrate_arg = DeclareLaunchArgument(
        'stm32_baudrate', default_value=TextSubstitution(text='115200'),
        description='STM32控制器通信波特率'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value=TextSubstitution(text='/dev/ttyUSB0'),
        description='RPLIDAR雷达连接的串口设备'
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame', default_value=TextSubstitution(text='laser'),
        description='RPLIDAR的坐标系名称'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value=TextSubstitution(text='false'),
        description='是否使用仿真时间'
    )
    
    # 获取RViz配置文件路径
    rviz_config_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'rviz',
        'slam.rviz'
    )

    return LaunchDescription([
        # 参数声明
        stm32_port_arg, stm32_baudrate_arg, lidar_port_arg, 
        lidar_frame_arg, use_sim_time_arg,
        
        # STM32 键盘控制节点
        Node(
            package='stm32_keyboard_control',
            executable='keyboard_control_node',
            name='keyboard_control_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('stm32_port'),
                'baudrate': LaunchConfiguration('stm32_baudrate'),
                'command_interval': 0.05
            }]
        ),
        
        # RPLIDAR 节点
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_port'),
                'frame_id': LaunchConfiguration('lidar_frame'),
                'inverted': False,
                'angle_compensate': True,
            }]
        ),
        
        # SLAM Toolbox（同步模式）
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'config',
                    'mapper_params_online_async.yaml'
                )
            }]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])