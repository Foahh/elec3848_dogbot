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
from launch_ros.actions import Node

def generate_launch_description():
    
    imu_filter_node = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        output="screen",
        parameters=[
            {
                "use_mag": True,
                "do_bias_estimation": True,
                "do_adaptive_gain": True,
                "gain_acc": 0.01,
                "gain_mag": 0.01,
                "publish_tf": False,
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
