import time
import serial


ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.flush()

while True:
    ser.write(b'<MOT-CCO|50>\n')
    time.sleep(1)
    ser.write(b'<MOT-CCW|50>\n')
    time.sleep(1)
    