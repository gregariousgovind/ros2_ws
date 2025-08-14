from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('a_star_planner_gazebo')
    params_file = os.path.join(pkg_share, 'params.yaml')

    return LaunchDescription([
        Node(
            package='a_star_planner_gazebo',
            executable='a_star_node',
            name='a_star_node',
            output='screen',
            parameters=[{'use_sim_time': True}, params_file]
        )
    ])
