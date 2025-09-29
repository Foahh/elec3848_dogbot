import numpy as np
import csv
import time
import serial

def save_csv_file(file_path):
    arduino_port = "/dev/arduino"
    arduino_baudrate = 115200
    with serial.Serial(arduino_port, arduino_baudrate) as arduino_serial:
        with open(file_path, mode="w", newline="") as file:
            csv_writer = csv.writer(file)
            start_time = time.time()
            while time.time() - start_time < 60:
                data = arduino_serial.readline().decode("utf-8").strip()
                try:
                    print(data)
                    csv_writer.writerow(map(int, data.split(",")))
                except ValueError as e:
                    print(f"Invalid data format: {data}; {e}")


def read_csv_file(file_path):
    rps_data = [[], [], [], []]
    time_interval = []
    try:
        with open(file_path, "r") as csvfile:
            reader = csv.reader(csvfile)
            for k, row in enumerate(reader):
                if k < 1000:
                    continue
                ms = float(row[0])
                time_interval.append(ms)
                for i in range(4):
                    x = abs(float(row[i + 1]))
                    rps_data[i].append(np.true_divide(x, ms))
    finally:
        return rps_data, time_interval

csv_file_path = "data.csv"
# save_csv_file(csv_file_path)
pulse_data, time_interval = read_csv_file(csv_file_path)

print(f"time: {np.average(time_interval):.6f}\n")
for i in range(len(pulse_data)):
    print(f"rps{i}: {np.average(pulse_data[i])/0.001:.6f}")
