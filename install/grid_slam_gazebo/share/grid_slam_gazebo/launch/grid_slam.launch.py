from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # GridSLAM parameters
    slam_params = os.path.join(
        get_package_share_directory('grid_slam_gazebo'),
        'params.yaml'
    )

    # Edited bridge YAML
    bridge_yaml = os.path.join(
        get_package_share_directory('my_turtlebot3_gazebo'),
        'config',
        'turtlebot3_burger_bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='grid_slam_gazebo',
            executable='grid_slam',
            name='grid_slam',
            output='screen',
            parameters=[slam_params]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            arguments=['--ros-args', '-p', f'config_file:={bridge_yaml}']
        )
    ])
