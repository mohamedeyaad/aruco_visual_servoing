from setuptools import setup
import os
from glob import glob

package_name = 'aruco_visual_servoing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Eyad',
    maintainer_email='mooeyad@gmail.com',
    description='Autonomous visual servoing with ArUco detection, Gazebo assets, and ROS 2 launch files.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_visual_servoing_node = aruco_visual_servoing.visual_servoing_node:main',
            'aruco_detector_node = aruco_visual_servoing.aruco_detector_node:main',
        ],
    },
)
