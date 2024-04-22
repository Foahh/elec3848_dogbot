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

import rclpy
import math
from rclpy.node import Node
from mpu6050 import mpu6050
from sensor_msgs.msg import Imu


class IMUPublisher(Node):
    def __init__(self):
        super().__init__("DogbotIMUPublisherNode")
        self.logger = self.get_logger()

        self.raw_publisher = self.create_publisher(Imu, "imu/data_raw", 10)

        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

        self.imu = mpu6050(0x68, 0)
        self.imu.set_accel_range(mpu6050.ACCEL_RANGE_4G)
        self.imu.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)
        self.raw_msg = Imu()

    def callback(self):
        accel_data = self.imu.get_accel_data(True)
        gyro_data = self.imu.get_gyro_data()

        self.raw_msg.header.stamp = self.get_clock().now().to_msg()
        self.raw_msg.header.frame_id = "imu_link"
        self.raw_msg.linear_acceleration.x = accel_data["x"]
        self.raw_msg.linear_acceleration.y = accel_data["y"]
        self.raw_msg.linear_acceleration.z = accel_data["z"]
        self.raw_msg.angular_velocity.x = gyro_data["x"] * math.pi / 180.0
        self.raw_msg.angular_velocity.y = gyro_data["y"] * math.pi / 180.0
        self.raw_msg.angular_velocity.z = gyro_data["z"] * math.pi / 180.0
        self.raw_publisher.publish(self.raw_msg)


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
