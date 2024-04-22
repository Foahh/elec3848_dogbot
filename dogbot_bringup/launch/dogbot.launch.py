# Copyright 2024 Long Liangmao
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

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dogbot_hardware"),
                    "launch",
                    "dogbot.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "gui": "false",
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dogbot_bringup"),
                    "launch",
                    "slam.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("dogbot_bringup"),
                    "launch",
                    "mpu6050.launch.py",
                ]
            )
        )
    )

    ekf_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("dogbot_bringup"),
                    "config",
                    "ekf_filter.yaml",
                ]
            )
        ],
    )

    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": PathJoinSubstitution(
                [FindPackageShare("dogbot_bringup"), "config", "navigation.yaml"]
            ),
        }.items(),
    )

    server_node = Node(
        package="dogbot_server",
        executable="dogbot_server",
        output="screen",
    )

    nodes = [
        hardware_launch,
        imu_launch,
        slam_launch,
        ekf_localization_node,
        server_node,
    ]

    return LaunchDescription(nodes)
