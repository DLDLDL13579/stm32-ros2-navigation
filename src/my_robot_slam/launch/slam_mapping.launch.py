#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 参数声明
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB2')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    enable_tf_monitor = LaunchConfiguration('enable_tf_monitor', default='False')
    use_ekf_odometry = LaunchConfiguration('use_ekf_odometry', default='True')
    
    # 机器人参数
    wheel_base = LaunchConfiguration('wheel_base', default='0.18')   # 修正：之前代码中轴距是 0.18
    wheel_track = LaunchConfiguration('wheel_track', default='0.17')  # 修正：之前代码中轮距是 0.17
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0335')
    
    # ===【关键修正】===
    # 这里的默认值必须更新为 2550.0，否则会覆盖节点代码中的正确设置
    pulses_per_revolution = LaunchConfiguration('pulses_per_revolution', default='1320.0')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                            description='STM32串口设备'),
        DeclareLaunchArgument('lidar_port', default_value=lidar_port,
                            description='雷达串口设备'),
        DeclareLaunchArgument('enable_tf_monitor', default_value=enable_tf_monitor,
                            description='是否启用TF监控节点'),
        DeclareLaunchArgument('use_ekf_odometry', default_value=use_ekf_odometry,
                            description='是否使用EKF融合里程计'),
        DeclareLaunchArgument('wheel_base', default_value=wheel_base,
                            description='前后轮轴距'),
        DeclareLaunchArgument('wheel_track', default_value=wheel_track,
                            description='左右轮轮距'),
        DeclareLaunchArgument('wheel_radius', default_value=wheel_radius,
                            description='车轮半径'),
        DeclareLaunchArgument('pulses_per_revolution', default_value=pulses_per_revolution,
                            description='每转脉冲数'),
        
        # 1. 启动键盘控制节点（包含四轮差速EKF融合功能）
        Node(
            package='stm32_keyboard_control',  # 请确保包名正确
            executable='keyboard_control_node', 
            name='keyboard_control_node',
            parameters=[{
                'serial_port': serial_port,
                'baudrate': 115200,
                'wheel_base': wheel_base,
                'wheel_track': wheel_track,
                'wheel_radius': wheel_radius,
                'pulses_per_revolution': pulses_per_revolution,
                'ekf_frequency': 50.0,
                'publish_tf': True,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'imu_frame_id': 'imu_link',
                'skid_steer_slip_factor': 1.6, # 建议显式添加滑移系数，方便调整
                'key_control_enabled': True    # 对应您Python代码中的参数名
            }],
            output='screen',
            respawn=True,
            respawn_delay=2
        ),
        
        # 2. 雷达节点
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 1000000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
                'use_scan_time': False
            }],
            output='screen',
            respawn=True,
            respawn_delay=2
        ),
        
        # 3. SLAM工具箱节点
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory('my_robot_slam'),
                    'config', 'my_slam_params.yaml'
                ])
            ],
            output='screen',
            remappings=[
                ('/odom', '/odom'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        ),
        
        # 4. RViz2节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                get_package_share_directory('my_robot_slam'),
                'rviz', 'slam_mapping.rviz'
            ])],
            output='screen'
        ),
        
        # 5. 静态TF变换 - 雷达到base_link
        # 这里的 0.1, 0, 0.15 是雷达相对于小车中心的安装位置(x,y,z)，请确保符合实际
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link',
            arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        # 6. 静态TF变换 - IMU到base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link',
            arguments=['0.05', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),
        
        # 7. TF监控节点（调试用）
        Node(
            package='tf2_ros',
            executable='tf2_monitor',
            name='tf_monitor',
            arguments=['map', 'base_link'],
            output='screen',
            condition=IfCondition(enable_tf_monitor)
        )
    ])