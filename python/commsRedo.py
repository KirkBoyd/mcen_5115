#!/usr/bin/env python3
import serial
import time

## modifying example from roboticsbackend.com/raspberry-pi-arduino-serial-communication
# retyped by Kirk Boyd
# last modified Nov 16, 2021
#i2c = busio.I2C(board.SCL, board.SDA)
if __name__ == '__main__':
    try:
        ser1 = serial.Serial('/dev/ttyACM0',38400,timeout=1,write_timeout=1)
        ser = serial.Serial('/dev/ttyACM3',38400,timeout=1,write_timeout=1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        i=1
        time.sleep(1)
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8','ignore').rstrip()
                print(line)
            if ser.out_waiting == 0:
                #print("waiting")
                packet = "<MOT|"+str(i)+"-0-0-255-1-1-1-1-0-0-0-0>\n"
                ser.write(packet.encode('utf-8'))
                #ser.write(b"<MOT|255-255-255-255-1-1-1-1-0-0-0-0>\n")
                i = i+1
                
                if (i % 10) == 0:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    ser1.reset_input_buffer()
                    ser1.reset_output_buffer()
                
    except KeyboardInterrupt:
        ser.write(b"<STP|>")
        print("turds")