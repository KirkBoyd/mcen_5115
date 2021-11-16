#!/usr/bin/env python3
import serial
import time

## modifying example from roboticsbackend.com/raspberry-pi-arduino-serial-communication
# retyped by Kirk Boyd
# last modified Nov 16, 2021

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    ser.reset_input_buffer()
    
    while True:
        ser.write(b"<MOT|255-255-255-255-1-1-1-1-0-0-0-0>\n")
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(1)
    
