from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    my_pkg_dir = get_package_share_directory('my_turtlebot3_gazebo')

    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'empty.world'
            ),
            description='Path to the world file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_pkg_dir, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),
    ])
