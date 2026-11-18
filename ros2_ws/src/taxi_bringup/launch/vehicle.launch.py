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
        )
    ])