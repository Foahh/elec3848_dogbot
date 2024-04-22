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
import time
from icm20948 import ICM20948  # https://github.com/pimoroni/icm20948-python
from smbus import SMBus
from sensor_msgs.msg import MagneticField, Imu

GYRO_FS_SEL_0 = 250
GYRO_FS_SEL_1 = 500
GYRO_FS_SEL_2 = 1000
GYRO_FS_SEL_3 = 2000

ACCEL_FS_SEL_0 = 2
ACCEL_FS_SEL_1 = 4
ACCEL_FS_SEL_2 = 8
ACCEL_FS_SEL_3 = 16


class IMUPublisher(Node):
    def __init__(self):
        super().__init__("DogbotIMUPublisherNode")
        self.logger = self.get_logger()

        self.raw_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_publisher = self.create_publisher(MagneticField, "imu/mag", 10)

        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

        self.bus = SMBus(0)
        time.sleep(1)
        self.imu = ICM20948(i2c_bus=self.bus)
        self.raw_msg = Imu()
        self.mag_msg = MagneticField()
        self.imu.set_accelerometer_full_scale(ACCEL_FS_SEL_1)
        self.imu.set_gyro_full_scale(GYRO_FS_SEL_1)

    def callback(self):
        mx, my, mz = self.imu.read_magnetometer_data()
        ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro_data()

        timestamp = self.get_clock().now().to_msg()

        self.raw_msg.header.stamp = timestamp
        self.raw_msg.header.frame_id = "imu_link"
        self.raw_msg.linear_acceleration.x = ax * 9.81
        self.raw_msg.linear_acceleration.y = ay * 9.81
        self.raw_msg.linear_acceleration.z = az * 9.81
        self.raw_msg.angular_velocity.x = gx * math.pi / 180.0
        self.raw_msg.angular_velocity.y = gy * math.pi / 180.0
        self.raw_msg.angular_velocity.z = gz * math.pi / 180.0
        self.raw_publisher.publish(self.raw_msg)

        self.mag_msg.header.stamp = timestamp
        self.mag_msg.header.frame_id = "imu_link"
        self.mag_msg.magnetic_field.x = mx * 1e-6
        self.mag_msg.magnetic_field.y = my * 1e-6
        self.mag_msg.magnetic_field.z = mz * 1e-6
        self.mag_publisher.publish(self.mag_msg)


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
