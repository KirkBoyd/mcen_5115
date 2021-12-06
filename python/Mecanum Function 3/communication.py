import numpy as np
import serial
from gpiozero import Button

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

def readBinary(binButton):
    if binButton.is_pressed:
        return 1
    else:
        return 0

def printBinary():
    
    # NEW BINARY STUFF #
    binaryPins = [17,27,22,13,19,26,20,21,25]
    binaryButtons = [0,0,0,0,0,0,0,0,0]
    binaryVals = [0,0,0,0,0,0,0,0,0]


    for i in range(9): # initialize buttons as pins
        binaryButtons[i] = Button(binaryPins[i],pull_up = None, active_state = True, hold_time = 0)
    
    binaryValStr = ""
    for i in range(9):
        binaryValStr = binaryValStr + str(readBinary(binaryButtons[i]))
        #print("Input " + str(i) + " is: " + str(binaryPins[i]) + " which is " + str(readBinary(binaryButtons[i])))
    
    binVal = (int(binaryValStr, 2))
    #print("binary vals: " + binaryValStr)
    #print("Final val is: " + str(binVal))
    
    return binVal