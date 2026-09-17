import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'diff_drive_robot'
    rviz_config = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'map_path.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_map_path',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([rviz_node])
