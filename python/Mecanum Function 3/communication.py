import numpy as np
import serial
def push(robot,motorSerial): #pushes data TO the arduino from the pi (Complete)
    speeds = ""
    directions = ""
    if motorSerial.out_waiting < 10:
        for i in range(4):
            speeds = speeds + str(robot.speeds[i])
            directions = directions + str(robot.directions[i])
            if i < 3:
                speeds = speeds + "-"
                directions = directions + "-"
                
        packetToSend = "<MOT|" + speeds + "-" + directions + ">\n"

        try:
            x = 0
            #motorSerial.write(packetToSend.encode('utf-8'))
            #print('Sent:' + packetToSend)
        except serial.serialutil.SerialTimeoutException:
            motorSerial.reset_output_buffer
            motorSerial.reset_output_buffer
            return
    else: motorSerial.reset_output_buffer