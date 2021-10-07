import serial
import time

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    
    while True:
        if ser.inWaiting():
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
        else:
            ser.write(input().encode('utf-8'))
        
        time.sleep(1)