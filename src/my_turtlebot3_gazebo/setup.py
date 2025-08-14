from setuptools import setup
import os
from glob import glob

package_name = 'my_turtlebot3_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ament package resource
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files (add any new launch files here)
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # config files (YAML)
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        # world files
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Custom TurtleBot3 Gazebo package',
    license='Apache License 2.0',
    tests_require=['pytest'],
)
