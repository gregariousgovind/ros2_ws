from setuptools import setup
import os
package_name = 'a_star_planner_gazebo'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/a_star.launch.py']),
        ('share/' + package_name, [os.path.join(package_name, 'params.yaml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='A* planner with RViz goal + simple follower',
    license='MIT',
    entry_points={'console_scripts': [
        'a_star_node = a_star_planner_gazebo.a_star_node:main'
    ]},
)
