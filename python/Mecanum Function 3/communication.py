import numpy as np
import serial

def pull(world,motorSerial,radioSerial):
    if (motorSerial.in_waiting > 0): 
        try:
            inPacket = motorSerial.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
            world.parseImu(inPacket)
        except UnicodeDecodeError:
            print("Invalid Packet")
            return world
    if (radioSerial.in_waiting > 0):
        try:
            inPacket = radioSerial.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
            world.parseRadio(inPacket)
        except UnicodeDecodeError:
            print("Invalid Packet")
            return world
    return world

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
            motorSerial.write(packetToSend.encode('utf-8'))
            #print('Sent:' + packetToSend)
        except serial.serialutil.SerialTimeoutException:
            motorSerial.reset_output_buffer
            motorSerial.reset_output_buffer
            return
    else: motorSerial.reset_output_buffer