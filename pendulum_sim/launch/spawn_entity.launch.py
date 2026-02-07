from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    # Spawn the pendulum entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'Pendulum'],
        output='screen'
    )

    # ros2_control spawners
    omni_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['stepper'],
        output='screen'
    )

    return LaunchDescription([
        spawn_entity,
        omni_controller_spawner
    ])
