from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the share directory for turtlebot3_gazebo (system or workspace)
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # World file launch argument
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(turtlebot3_gazebo_dir, 'worlds', 'empty.world'),
            description='Path to the world file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),
    ])
