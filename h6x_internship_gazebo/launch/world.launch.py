"""Launch world."""
# Copyright 2022 HarvestX Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
)
from launch_ros.descriptions import ComposableNode

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
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(pkg_line_trace, 'rviz', 'line_trace.rviz')]
    )

    judge = ComposableNodeContainer(
        name='judge_container',
        namespace='judge',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='h6x_internship_gazebo',
                plugin='JudgeGoal',
                name='judge_goal',
            ),
            ComposableNode(
                package='h6x_internship_gazebo',
                plugin='GameMaster',
                name='game_master',
            ),
            ComposableNode(
                package='h6x_internship_gazebo',
                plugin='JudgeDeviation',
                name='judge_deviation',
            ),
        ])

    return LaunchDescription([
        gzclient_cmd,
        gzserver_cmd,
        joint_state_publisher_node,
        rviz,
        judge
    ])
