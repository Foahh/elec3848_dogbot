import serial
import time

with serial.Serial() as ser:
    ser.baudrate = 115200
    ser.port = '/dev/ttyUSB0'
    ser.timeout = 1
    ser.open()
    
    command = "<D>"
    while True:
        val = ser.write(command.encode(encoding = 'ascii', errors = 'strict'))
        ser.flush()
        time.sleep(0.5)
        print(ser.read(100))