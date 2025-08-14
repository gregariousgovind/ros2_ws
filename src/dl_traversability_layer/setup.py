from setuptools import setup
import os
package_name = 'dl_traversability_layer'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/traversability.launch.py']),
        ('share/' + package_name, [os.path.join(package_name, 'params.yaml')]),
    ],
    install_requires=['setuptools','tensorflow==2.15.*','numpy'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='DL traversability layer for A*',
    license='MIT',
    entry_points={'console_scripts': [
        'traversability = dl_traversability_layer.traversability_node:main'
    ]},
)
