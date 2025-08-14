from setuptools import setup
import os

package_name = 'grid_slam_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ROS 2 package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/grid_slam.launch.py',
            'launch/gazebo_tb3_world.launch.py'
        ]),
        # Parameters file
        ('share/' + package_name, [os.path.join(package_name, 'params.yaml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='2D Grid-Based SLAM (log-odds) for ROS 2 + Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'grid_slam = grid_slam_gazebo.grid_slam_node:main'
        ],
    },
)
