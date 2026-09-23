from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('rb2301_tutorial')

    return LaunchDescription([
        Node(
            package='rb2301_tutorial',
            executable='prms',
            name='parameters',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    package_share,
                    'params',
                    'prms.yaml',
                ])
            ],
        ),
    ])
