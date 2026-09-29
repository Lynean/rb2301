from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # initialize launch description
    ld = LaunchDescription()

    # 1. PACKAGE INSTALLATION DIRECTORIES 
    pkg_rb2301_tutorial = FindPackageShare('rb2301_tutorial')
    pkg_rb2301_bringup = FindPackageShare('rb2301_bringup')
    # 2. LAUNCH ARGUMENTS
    arg_world = DeclareLaunchArgument(
        'world', 
        default_value='test.sdf',
        description='test world'
    )
    ld.add_action(arg_world)
    # 3. LAUNCH OTHER LAUNCH FILES
    launch_sim_launch_bringup= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_rb2301_bringup, 'launch', 'sim.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items()
    )
    ld.add_action(launch_sim_launch_bringup)
    # 4. RUN NODES

    return ld
