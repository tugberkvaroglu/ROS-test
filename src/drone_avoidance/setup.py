from setuptools import setup
from glob import glob
import os

package_name = 'drone_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Object Avoidance Simulation using ROS2 and Gazebo',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'object_avoidance = drone_avoidance.object_avoidance:main',  # Entry point directly to main function
        ],
    },
)
