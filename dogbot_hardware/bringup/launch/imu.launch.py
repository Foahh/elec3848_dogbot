# Copyright 2020 ros2_control Development Team
#
# Modified by Long Liangmao in 2024
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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mag",
            default_value="true",
            description="Use magnetometer data.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "do_bias_estimation",
            default_value="true",
            description="Do bias estimation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "do_adaptive_gain",
            default_value="true",
            description="Do adaptive gain.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gain_acc",
            default_value="0.01",
            description="Gain for accelerometer.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gain_mag",
            default_value="0.01",
            description="Gain for magnetometer.",
        )
    )

    # Initialize Arguments
    use_mag = LaunchConfiguration("use_mag")
    do_bias_estimation = LaunchConfiguration("do_bias_estimation")
    do_adaptive_gain = LaunchConfiguration("do_adaptive_gain")
    gain_acc = LaunchConfiguration("gain_acc")
    gain_mag = LaunchConfiguration("gain_mag")

    imu_filter_node = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        output="screen",
        parameters=[
            {
                "use_mag": use_mag,
                "do_bias_estimation": do_bias_estimation,
                "do_adaptive_gain": do_adaptive_gain,
                "gain_acc": gain_acc,
                "gain_mag": gain_mag,
            }
        ],
    )

    imu_publisher_node = Node(
        package="dogbot_imu",
        executable="imu_publisher",
        output="screen",
    )

    nodes = [
        imu_filter_node,
        imu_publisher_node,
    ]

    return LaunchDescription(nodes)
