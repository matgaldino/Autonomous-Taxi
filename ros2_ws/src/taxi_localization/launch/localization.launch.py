from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config = PathJoinSubstitution([
        FindPackageShare('taxi_localization'),
        'config',
        'ekf.yaml'
    ])

    return LaunchDescription([

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_transform',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '1.57079632679',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link'
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[config],
            output='screen'
        )
    ])