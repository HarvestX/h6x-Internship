import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_line_trace = get_package_share_directory('h6x_internship_gazebo')

    world = os.path.join(
        get_package_share_directory('h6x_internship_gazebo'),
        'worlds',
        'env_line_room.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, "verbose": "true"}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    joint_state_publisher_node = Node(
        package ='joint_state_publisher',
        executable ='joint_state_publisher',
        name ='joint_state_publisher'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_line_trace, 'rviz', 'line_trace.rviz')]
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        joint_state_publisher_node,
        rviz
    ])
