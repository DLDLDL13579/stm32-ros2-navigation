from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 RPLIDAR S2 节点
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 1000000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }]
        ),

        # 添加 base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf_pub',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        # 添加 odom → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # 启动 SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'map_file_name': '',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
            }],
        ),

        # 启动 RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/opt/ros/humble/share/slam_toolbox/rviz/slam_toolbox.rviz']
        ),
    ])
