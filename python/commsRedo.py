#!/usr/bin/env python3
import serial
import time

## modifying example from roboticsbackend.com/raspberry-pi-arduino-serial-communication
# retyped by Kirk Boyd
# last modified Nov 16, 2021

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    i=100
    time.sleep(1)
    
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8','ignore').rstrip()
            print(line)
        if ser.out_waiting == 0:
            packet = "<MOT|"+str(i)+"-255-255-255-1-1-1-1-0-0-0-0>\n"
            ser.write(packet.encode('utf-8'))
            #ser.write(b"<MOT|255-255-255-255-1-1-1-1-0-0-0-0>\n")
            i = i+1
            
        #time.sleep(1)

'''
# Step1
    Sending this:
        ser.write(b"<MOT|255-255-255-255-1-1-1-1-0-0-0-0>\n")
        line = ser.readline().decode('utf-8').rstrip()
        print(line)

    Yielded this successfully:
        You sent me: <MOT|255-255-255-255-1-1-1-1-0-0-0-0>

# Step2
    Going to modify .ino file so that it separates out things from that string
'''
