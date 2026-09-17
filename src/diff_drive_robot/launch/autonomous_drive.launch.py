from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')

    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Open RViz for map/path visualization')

    pkg_path = get_package_share_directory('diff_drive_robot')
    costmap_path = os.path.join(pkg_path, 'config', 'costmap.yaml')


    odrive_node = Node(
        package='diff_drive_robot',
        executable='odrive_node',
        name='odrive_node',
        output='screen',
        parameters=[{
            'velocity_scale': 1.0,
            'max_velocity': 2.0,
            'use_sim_time': True
        }]
    )

    line_node = Node(
        package='diff_drive_robot',
        executable='detect_line',
        name='detect_line_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image',
            'kp': 0.004,
            'base_speed': 1.4,
            'lookahead_gain': 1.5,
            'angle_gain': 0.5,
            'min_contour_area': 300,
            'min_padding': 80,
            'search_speed': 0.3,
            'search_turn': -0.3,
            'line_lost_time': 0.5,
            'follow_left': True,
            'obstacle_threshold': 0.5,
            'use_sim_time': True,
        }]
    )

    behavior_node = Node(
        package='diff_drive_robot',
        executable='behavior_node',
        name='behavior_node',
        output='screen',
        parameters=[{
            'obstacle_distance': 0.6,
            'avoid_strength': 1.5,
            'front_avoid_strength': 1.0,
            'forward_speed_scale': 0.75,
            'follow_left': True,
            'critical_distance': 1.5,
            'use_map_avoidance': False,
            'use_sim_time': True,
        }]
    )

    path_trail_node = Node(
        package='diff_drive_robot',
        executable='path_trail',
        name='path_trail_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ekf_params_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path, {'use_sim_time': True}]
    )

    # Emlid RS+ GPS node for high-precision GNSS input
    emlid_node = Node(
        package='diff_drive_robot',
        executable='emlid_gps_node',
        name='emlid_gps_node',
        output='screen',
        parameters=[{
            'emlid_host': '192.168.1.100',
            'emlid_port': 2101,
            'protocol': 'tcp',
            'frame_id': 'gps',
            'origin_lat': 47.47895,
            'origin_lon': 19.057785,
            'origin_alt': 0.0,
            'publish_local_pose': True,
            'use_sim_time': True,
        }]
    )

    waypoint_params_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')
    waypoint_driver = Node(
        package='diff_drive_robot',
        executable='waypoint_driver',
        name='waypoint_driver',
        output='screen',
        parameters=[waypoint_params_path, {'use_sim_time': True}]
    )

    slam_toolbox_params_path = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_params_path, {'use_sim_time': True}]
    )

    costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',        
        parameters=[costmap_path, {'use_sim_time': True}]
    )

    rviz_map_path_config_file = os.path.join(pkg_path, 'rviz', 'map_path.rviz')
    rviz2_map_path = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_map_path',
            arguments=['-d', rviz_map_path_config_file],
            output='screen',
            parameters=[{'use_sim_time': True}],
        )]
    )

    # Also launch the full RViz with camera, robot model, map, and lidar
    rviz_full_config_file = os.path.join(pkg_path, 'rviz', 'bot.rviz')
    rviz2_full = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_full',
            arguments=['-d', rviz_full_config_file],
            output='screen',
            parameters=[{'use_sim_time': True}],
        )]
    )

    return LaunchDescription([
        declare_rviz,
        odrive_node,
        line_node,
        behavior_node,
        path_trail_node,
        robot_localization_node,
        emlid_node,
        waypoint_driver,
        slam_toolbox_node,
        costmap,
        rviz2_full,
        rviz2_map_path
    ])