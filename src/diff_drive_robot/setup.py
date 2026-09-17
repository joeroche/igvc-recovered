from setuptools import setup
import os
from glob import glob

package_name = 'diff_drive_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='igvc',
    maintainer_email='igvc@todo.todo',
    description='Autonomous diff drive robot',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*')),

        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
    ],
    entry_points={
        'console_scripts': [
            'detect_line = diff_drive_robot.detect_line:main',
            'behavior_node = diff_drive_robot.behavior_node:main',
            'odrive_node = diff_drive_robot.odrive:main',
            'path_trail = diff_drive_robot.path_trail:main',
            'emlid_gps_node = diff_drive_robot.emlid_gps:main',
            'waypoint_driver = diff_drive_robot.waypoint_driver:main',
        ],
    },
)