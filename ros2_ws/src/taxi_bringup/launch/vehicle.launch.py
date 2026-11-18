from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('taxi_teleop'),
                    'launch',
                    'gamepad_teleop.launch.py'
                ])
            )
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('taxi_localization'),
                    'launch',
                    'localization.launch.py'
                ])
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_a2m8_launch.py'
                ])
            ),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '115200',
                'frame_id': 'laser'
            }.items()
        ),

        Node(
            package='taxi_serial_bridge',
            executable='serial_bridge',
            output='screen'
        ),

        Node(
            package='taxi_odometry',
            executable='wheel_odometry',
            output='screen',
            parameters=[{'publish_tf': False}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_transform',
            arguments=[
                '--x', '0.2918',
                '--y', '0.0',
                '--z', '0.0727',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '-2.50',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ]
        )
    ])