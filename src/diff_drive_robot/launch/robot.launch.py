import os

from click import edit
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

def generate_launch_description():

    # Package name
    package_name='diff_drive_robot'

    # Launch configurations
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    # use_ros2_control = LaunchConfiguration("use_ros2_control")

    declare_use_ros2_control = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="Use ros2_control instead of Gazebo diff drive"
    )

    # Path to default world -                                                       change this file path 
    world_path = os.path.join(get_package_share_directory(package_name),'worlds', 'circle.world')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')

    # Launch Robot State Publisher Node
    urdf_path = os.path.join(get_package_share_directory(package_name),'urdf','robot.urdf')
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # Launch the gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Always launch the gazebo client to visualize the simulation
    gazebo_client = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                    )]), launch_arguments={'gz_args': '-g '}.items()
    )

    # Run the spawner node from the gazebo_ros package. 
    spawn_diff_bot = Node(
                        package='ros_gz_sim', 
                        executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'diff_bot',
                                   '-x', '-6.0', #added @ 2026 comp
                                   '-y', '-42.6', #prev =  10.75
                                   '-z', '0.12'], 
                        output='screen'
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',],
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch the full RViz (camera, robot model, map, lidar)
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen',
                    parameters=[{'use_sim_time': True}],)]
    )

    # Launch only the dedicated map/path RViz (single 2D canvas)
    rviz_map_path_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'map_path.rviz')
    rviz2_map_path = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_map_path',
                    arguments=['-d', rviz_map_path_config_file],
                    output='screen',
                    parameters=[{'use_sim_time': True}],)]
    )

    # Launch a second Rviz window for map and path tracing
    rviz_map_path_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'map_path.rviz')
    rviz2_map_path = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_map_path',
                    arguments=['-d', rviz_map_path_config_file],
                    output='screen',
                    parameters=[{'use_sim_time': True}],)]
    )


    autonomous_drive = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(package_name),
            'launch',
            'autonomous_drive.launch.py'
        ))
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments
        declare_rviz,
        declare_world,
        declare_use_ros2_control,

        load_joint_state_broadcaster,
        load_diff_drive_controller,
        # Launch the nodes
        autonomous_drive,
        rviz2,
        rviz2_map_path,
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_diff_bot
    ])


