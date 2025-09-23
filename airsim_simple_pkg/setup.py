from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'airsim_simple_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource', 'maps'), glob('resource/maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bohang Liang',
    maintainer_email='user@example.com',
    description='A ROS2 package for AirSim integration providing map management and drone perception',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'airsim_perception_node = airsim_simple_pkg.airsim_perception_node:main',
        ],
    },
)
