from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dons_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # This will get ALL .py files in launch/
        ('share/' + package_name + '/config', glob('config/*.yaml')),  # Now the glob will work
        ('share/' + package_name + '/rviz', ['rviz/robot_view.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/dons_robot.urdf', 'urdf/dons_robot_gazebo.urdf']),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),  # Simpler - gets all STL files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='don',
    maintainer_email='dwilliestyle@gmail.com',
    description='ROS2 package for Don\'s RaspRover robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = dons_robot.esp32_bridge:main',
        ],
    },
)