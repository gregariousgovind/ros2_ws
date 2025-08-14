from setuptools import setup
import os
from glob import glob

package_name = 'markov_localization_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Package resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files (all .launch.py in launch/)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # RViz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Params file(s) at package root
        (os.path.join('share', package_name), glob('*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Grid-based Markov Localization for ROS 2 + Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Keep this matching your node's actual file and main()
            'markov_localization = markov_localization_gazebo.markov_localization:main'
        ],
    },
)
