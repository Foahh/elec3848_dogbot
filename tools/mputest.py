from icm20948 import ICM20948

if __name__ == "__main__":
    imu = ICM20948()
    imu.set_accelerometer_full_scale(8)
    imu.set_gyro_full_scale(500)

    while True:
        x, y, z = imu.read_magnetometer_data()
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()

        print(
            """
Accel: {:05.2f} {:05.2f} {:05.2f}
Gyro:  {:05.2f} {:05.2f} {:05.2f}
Mag:   {:05.2f} {:05.2f} {:05.2f}""".format(
                ax, ay, az, gx, gy, gz, x, y, z
            )
        )

        time.sleep(0.01)
