import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition


def generate_launch_description():
    # Get package share directory (Modify if your package name is different)
    # path = get_package_share_directory('pendulum_description')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    # use_rviz_path = LaunchConfiguration('use_rviz_path')
    # path_arg = DeclareLaunchArgument(
    #     name='use_rviz_path',
    #     default_value=os.path.join(
    #         path,
    #         'config',
    #         'rviz',
    #         'urdf_view.rviz'
    #     ),
    #     description='Path to the RViz configuration file'
    # )

    # RViz
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',

    #     output='screen',
    #     arguments=['-d', use_rviz_path],
    #     condition=IfCondition(use_rviz)
    # )

    # Launch!
    return LaunchDescription([
        # Arguments
        time_arg,
        # rviz_arg,
        # path_arg,

        # Nodes
        # rviz_node
    ])
