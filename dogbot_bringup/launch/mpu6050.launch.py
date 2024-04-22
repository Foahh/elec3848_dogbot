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
                "do_bias_estimation": True,
                "do_adaptive_gain": True,
                "gain_acc": 0.01,
                "gain_mag": 0.01,
                "publish_tf": False,
            }
        ],
        
    )

    imu_publisher_node = Node(
        package="mpu6050driver",
        executable="mpu6050driver",
        name="mpu6050driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "calibrate": True,

                "gyro_range": 0,
                "accel_range": 1,
                "dlpf_bandwidth": 2,
                "gyro_x_offset": 0.0,  # [deg/s]
                "gyro_y_offset": 0.0,  # [deg/s]
                "gyro_z_offset": 0.0,  # [deg/s]
                "accel_x_offset": 0.0,  # [m/s²]
                "accel_y_offset": 0.0,  # [m/s²]
                "accel_z_offset": 0.0,  # [m/s²]
                "frequency": 200,  # [Hz]
            }
        ],
        remappings=[ 
            ("/imu/data_raw", "/imu/data_raw"),
            ("/imu/mag", "/imu/mag"),
        ],
    )

    nodes = [
        imu_filter_node,
        imu_publisher_node,
    ]

    return LaunchDescription(nodes)
