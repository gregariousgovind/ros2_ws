from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('markov_localization_gazebo')
    params_file = os.path.join(pkg_share, 'params.yaml')

    return LaunchDescription([
        Node(
            package='markov_localization_gazebo',
            executable='markov_localization',
            name='markov_localization',
            output='screen',
            parameters=[{'use_sim_time': True}, params_file]
        )
    ])
