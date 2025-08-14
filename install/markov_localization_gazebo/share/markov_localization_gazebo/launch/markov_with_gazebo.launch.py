from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # TurtleBot3 Gazebo launch (change if using custom robot/world)
    tb3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Markov Localization node
    markov_node = Node(
        package='markov_localization_gazebo',
        executable='markov_localization',  # adjust if different
        name='markov_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            os.path.join(
                get_package_share_directory('markov_localization_gazebo'),
                'params.yaml'
            )
        ]
    )

    # RViz2 preloaded with config
    rviz_config_path = os.path.join(
        get_package_share_directory('markov_localization_gazebo'),
        'rviz',
        'markov_localization.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        tb3_gazebo_launch,
        markov_node,
        rviz_node
    ])
