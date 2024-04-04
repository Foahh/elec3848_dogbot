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

import qwiic_icm20948

from sensor_msgs.msg import MagneticField, Imu


class IMUPublisher(Node):

    def __init__(self):
        super().__init__("imu_publisher")
        self.logger = self.get_logger()

        self.raw_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_publisher = self.create_publisher(MagneticField, "imu/mag", 10)

        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

        self.imu = qwiic_icm20948.QwiicIcm20948()
        if not self.imu.connected:
            self.logger.error("IMU not connected. Please check connection.")
        self.imu.begin()

        self.imu.setFullScaleRangeAccel(qwiic_icm20948.gpm4)
        self.accel_f = 8192.0  # 8192 LSB/g

        self.imu.setFullScaleRangeGyro(qwiic_icm20948.dps500)
        self.gyro_f = 65.5  # 65.5 LSB/dps
        
        self.mag_f = 0.15  # 0.15 uT/LSB

    def callback(self):
        if self.imu.dataReady():
            try:
                self.imu.getAgmt()
            except Exception as e:
                self.logger.error(f"Error reading data from IMU: {e}")

            timestamp = self.get_clock().now().to_msg()

            raw_msg = Imu()
            raw_msg.header.stamp = timestamp
            raw_msg.header.frame_id = "imu_link"
            raw_msg.linear_acceleration.x = self.imu.axRaw / self.accel_f * 9.80665
            raw_msg.linear_acceleration.y = self.imu.ayRaw / self.accel_f * 9.80665
            raw_msg.linear_acceleration.z = self.imu.azRaw / self.accel_f * 9.80665
            raw_msg.angular_velocity.x = self.imu.gxRaw / self.gyro_f * math.pi / 180.0
            raw_msg.angular_velocity.y = self.imu.gyRaw / self.gyro_f * math.pi / 180.0
            raw_msg.angular_velocity.z = self.imu.gzRaw / self.gyro_f * math.pi / 180.0
            self.raw_publisher.publish(raw_msg)

            mag_msg = MagneticField()
            mag_msg.header.stamp = timestamp
            mag_msg.header.frame_id = "imu_link"
            mag_msg.magnetic_field.x = self.imu.mxRaw * self.mag_f * 1e-6
            mag_msg.magnetic_field.y = self.imu.myRaw * self.mag_f * 1e-6
            mag_msg.magnetic_field.z = self.imu.mzRaw * self.mag_f * 1e-6
            self.mag_publisher.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
