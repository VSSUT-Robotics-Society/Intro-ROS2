from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    # Package shares (Modify if your package names are different)
    desc_pkg = get_package_share_directory('pendulum_description')
    sim_pkg = get_package_share_directory('pendulum_sim')

    # Arguments
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Optional world file path (.sdf / .world). Empty = default empty world.'
    )

    # Setup the launch file within Description for RViz and Robot State Publisher
    desc_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                desc_pkg,
                'launch',
                'urdf.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': 'true',
            'use_joint_state_publisher_gui': 'false',
            # 'use_rviz_path': os.path.join(
            #     sim_pkg,
            #     'config',
            #     'rviz',
            #     'gz_view.rviz'
            # )
        }.items()
    )

    # Setup Gazebo sim launch (no bots/entities spawned yet)
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r -v4 '), world],
            'on_exit_shutdown': 'true'  # Ensures shutdown of all nodes upon Gazebo exit
        }.items()
    )

    # Spawn the pendulum entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'Pendulum'],
        output='screen'
    )

    # ros_gz_bridge for topic bridging between ROS and Gazebo
    # Source: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=PathJoinSubstitution([
            sim_pkg,
            'config',
            'ros_gz_bridge.yaml'
        ]),
        create_own_container=False,  # Don't create container (standalone node)
        use_composition=False,       # Run as standalone node, not composed
        use_respawn=False,          # Don't respawn on crash
        log_level='info'            # Info-level logging for bridge diagnostics
        # Note: extra_bridge_params can be added here if additional runtime params needed
    )

    # Launch!
    return LaunchDescription([
        # Arguments
        world_arg,

        # Actions
        desc_file,
        gz_sim_launch,
        spawn_entity,
        ros_gz_bridge,
    ])
