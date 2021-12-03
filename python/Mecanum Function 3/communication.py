import numpy as np
import serial
def pull():
    pass

def parse():
    pass

def push(robot,motorSerial): #pushes data TO the arduino from the pi
    if motorSerial.out_waiting < 10:
        for i in range(4):
            motor = motor + str(robot.speeds[i])
            direction = direction + str(robot.directions[i])
            if i < 3:
                motor = motor + "-"
                outA1 = outA1 + "-"
                outA2 = outA2 + "-"
                
        packet = "<MOT|" + motor + "-" + outA1 + "-" + outA2 + ">\n"
        if connected:
            try:
                ser0.write(packet.encode('utf-8'))
                print('Sent:' + packet)
            except serial.serialutil.SerialTimeoutException:
                ser0.reset_output_buffer
                ser1.reset_output_buffer
                return
    else: motorSerial.reset_output_buffer