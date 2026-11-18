from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_config = PathJoinSubstitution([
        FindPackageShare('taxi_bringup'),
        'rviz',
        'vehicle.rviz'
    ])

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('taxi_description'),
                    'launch',
                    'description.launch.py'
                ])
            )
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])