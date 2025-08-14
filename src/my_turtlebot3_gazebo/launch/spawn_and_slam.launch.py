from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    pkg_slam = get_package_share_directory('grid_slam_gazebo')
    pkg_my = get_package_share_directory('my_turtlebot3_gazebo')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')

    # Paths to files
    slam_params = os.path.join(pkg_slam, 'params.yaml')
    bridge_yaml = os.path.join(pkg_my, 'config', 'turtlebot3_burger_bridge.yaml')
    world_file = os.path.join(pkg_my, 'worlds', 'my_custom_world.world')  # <- your world file

    # LaunchConfiguration for world
    world = LaunchConfiguration('world', default=world_file)

    return LaunchDescription([
        # Launch Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),

        # Start GridSLAM
        Node(
            package='grid_slam_gazebo',
            executable='grid_slam',
            name='grid_slam',
            output='screen',
            parameters=[slam_params]
        ),

        # Start parameter bridge with your YAML
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}']
        )
    ])
