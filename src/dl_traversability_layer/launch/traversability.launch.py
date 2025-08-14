from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('dl_traversability_layer')
    params_file = os.path.join(pkg_share, 'params.yaml')

    return LaunchDescription([
        Node(
            package='dl_traversability_layer',
            executable='traversability',
            name='traversability',
            output='screen',
            parameters=[{'use_sim_time': True}, params_file]
        )
    ])
