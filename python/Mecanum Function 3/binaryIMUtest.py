#Libraries
import numpy as np
import time
from numpy.core.numeric import ones
from gpiozero import Button
from gpiozero import LED
import serial
import signal

#Helper Files
import world
import kinematics
import debugging
import communication
import navigation

# GPIO Variables
team = 'green'
button = Button(16)
buttonGRN = Button(5)
buttonBLU = Button(6)
ledBLU = LED(23)
imuCounter = 0
ledGRN = LED(24)
radCounter = 0
# ledIMU = LED(25)
# ledRAD = LED(13)


# NEW BINARY STUFF #
binaryPins = [17,27,22,13,19,26,20,21,25]
binaryButtons = [0,0,0,0,0,0,0,0,0]
binaryVals = [0,0,0,0,0,0,0,0,0]


for i in range(9): # initialize buttons as pins
    binaryButtons[i] = Button(binaryPins[i])
    

def readBinary(binButton):
    if binButton.is_pressed:
        return 1
    else:
        return 0

def printBinary():
    binaryValStr = ""
    for i in range(9):
        binaryValStr = binaryValStr + str(readBinary(binaryButtons[i]))
        print("Input " + str(i) + " is: " + str(binaryPins[i]) + " which is " + str(readBinary(binaryButtons[i])))
    
    binVal = (int(binaryValStr, 2))
    print("Final val is: " + str(binVal))
    
# ARDUINO SAYS
# <177>
# Digital Pin: 0 is: 26 with value: 1 Black
# Digital Pin: 1 is: 27 with value: 0 Brown
# Digital Pin: 2 is: 28 with value: 1 Red
# Digital Pin: 3 is: 29 with value: 1 Orange
# Digital Pin: 4 is: 30 with value: 0 Yellow
# Digital Pin: 5 is: 31 with value: 0 Green
# Digital Pin: 6 is: 32 with value: 0 Blue
# Digital Pin: 7 is: 33 with value: 1 Purple
# Digital Pin: 8 is: 34 with value: 1 Grey

# WE GOT
# Input 0 is: 17 which is 0
# Input 1 is: 27 which is 1
# Input 2 is: 22 which is 0
# Input 3 is: 13 which is 0
# Input 4 is: 19 which is 0
# Input 5 is: 26 which is 1
# Input 6 is: 20 which is 1
# Input 7 is: 21 which is 0
# Input 8 is: 25 which is 1
# Final val is: 141
#Serial Communication Initialization and resetting
# serMotors = serial.Serial('/dev/ttyACM2',9600,write_timeout=.05,timeout=.5) #IMU and Motors
# serRadio = serial.Serial('/dev/ttyACM1',9600,write_timeout=.5,timeout=.5) #Radio
# time.sleep(1)
# 
# serMotors.flush()
# serMotors.reset_output_buffer()
# serMotors.reset_input_buffer()
# 
# serRadio.flush()
# serRadio.reset_output_buffer()
# serRadio.reset_input_buffer()
# connected = True

#Serial Communication functions
# def pull(world):
#     if (serMotors.in_waiting > 0): 
#         try:
#             #print("trying to read IMU packet")
#             inPacket = serMotors.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
#             #print(inPacket)
#             world.parseImu(inPacket)
#             #print("The f word")
#         except UnicodeDecodeError:
#             print("Invalid Packet")
#             return world
#     if (serRadio.in_waiting > 0):
#         try:
#             #print("trying to read mmmmmmmRadio packet")
#             inPacket = serRadio.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
#             world.parseRadio(inPacket)
#             #sprint(inPacket)
#         except UnicodeDecodeError:
#             #print("Invalid Packet")
#             return world
#     return world
# 
# def push(world): #pushes data TO the arduino from the pi (Complete)
#     robot = world.robot
#     speeds = ""
#     directions = ""
#     if serMotors.out_waiting ==0:
#         for i in range(4):
#             speeds = speeds + str(int(robot.speeds[i]))
#             directions = directions + str(int(robot.directions[i]))
#             if i < 3:
#                 speeds = speeds + "-"
#                 directions = directions + "-"
#                 
#         packetToSend = "<MOT|" + speeds + "-" + directions + "->\n"
# 
#         try:
#             serMotors.write(packetToSend.encode('utf-8'))
#             #print('Sent:' + packetToSend)
#         except serial.serialutil.SerialTimeoutException:
#             serMotors.reset_output_buffer
#             serMotors.reset_output_buffer
#             print("Push Serial Timeout")
#             return
#     else: serMotors.reset_output_buffer
#     
# def playSoccer():
#     pass
# 
# def testDebug():
#     try:
#         debugWorld = world.worldClass()
#         debugWorld = navigation.updateGoalPositions(debugWorld,300,50,0)
#         i = 1
#         while True:
#             debugWorld = pull(debugWorld)
#             debugWorld = kinematics.updateGoalSpeeds(debugWorld)
#             debugWorld = kinematics.updateMotorSpeeds(debugWorld)
#             debugging.printRobotCoords(debugWorld)
#             push(debugWorld)
#             #print("loop", i)
#             #i += 1
#     except KeyboardInterrupt:
#         print("turds")
#         serMotors.write(b"<STP|>")
#             
#     pass

if __name__ == '__main__': #Main Loop
    printBinary()
#     print('------------------------')
#     print('Waiting for start button')
#     print('------------------------')
#     while not button.value:
#         if buttonGRN.is_pressed:
#             team = 'green'
#             posTargetx = 192
#             posTargety = 499
#             posProtectx = 183 #Our goal
#             posProtecty = -26 #Our Goal
#             ledGRN.on()
#             ledBLU.off()
#         elif buttonBLU.is_pressed:
#             team = 'blue'
#             posTargetx = 183
#             posTargety = -26
#             posProtectx = 192 #Our goal
#             posProtecty = 499 #Our Goal
#             ledBLU.on()
#             ledGRN.off()
#     print("Started")
#     testDebug()
