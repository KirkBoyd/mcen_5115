import numpy as np
import serial
import time

posBallx = 1500
posBally = 2000
posOppx = 500.0
posOppy = 3200
posRobx = 700
posRoby = 150
posRobt = 0

ser = serial.Serial('COM5',9600) #Windows serial port
#ser = serial.Serial('\dev\ttyUSB*',9600) #Unix serial port

#List of Commands or functions
    #Input
        #Radio
            #Robot rob
            #Enemy ene
            #Ball bal
        #Pixy PIX
            #Obj obj
            #Size siz
            #X pix
            #Y piy
        #IMU
            #Vx vex
            #Vy vey
            #THETAz thz
    #Output
        #Motors
            #4 Motors MT0 MT1 MT2 MT3
            #rpm 0:999
            #inA1
            #inA2


def pull(): #pulls (or receives) data from the arduino on the pi
    global posRobx
    global posRoby
    global posRobt
    global posOppx
    global posOppy
    global posBallx
    global posBally


    START_MARKER = '<'  #marks the beginning of a data packet
    END_MARKER = '>'    #marks the end of a data packet
    COMMAND_SEP = '|'   #marks a separation between commands within a packet
    VALUE_SEP = '-'     #??

    receiving = False   #set to true when start marker is received, set to false when end marker is received
    commandReceived = False    #set to true when command separator is received (or if command buffer is full)

    cmdLen = 3 #length of command to recieve
    cmdBuffer = ""

    index = 0
    cmdIndex = 0
    if (ser.in_waiting > 0):
        packet = ser.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
        print(packet) #print what was received from the serial port
        if (isWhiteSpace(packet)):
            return # Ignore whitespace
        while index < len(packet): #step through each byte of the packet
            serialByte = packet[index] #the byte we are looking at is the one currently at index
            # print(serialByte)
            if (isWhiteSpace(serialByte)): #ignore whitespace again if found
                return
            if serialByte == START_MARKER: #if start marker is found
                receiving = True #record that a packet is beginning to be received
                commandReceived = True #record that a packet was in fact received
                index = index + 1
                continue
            if(receiving): #if looking for a packet
                serialByte = packet[index]
                if not commandReceived:
                    if (serialByte == COMMAND_SEP): #If the command separator is received
                        index = index + 1 #count forward one because there is not a command in this byte
                        commandReceived = True
                        continue
                    elif (serialByte == END_MARKER): #If end marker is reached
                        return
                    else:
                        index = index + 1
                else:
                    cmdBuffer = packet[index:index+3]
                    index = index + 4
                    if (cmdBuffer == "ROB"): #Check if the received string is "ROB"
                        print("ROB") #this is for position of the robot
                        while packet[index+cmdIndex] != VALUE_SEP:
                            cmdIndex = cmdIndex + 1
                        posRobx= int(packet[index:index+cmdIndex])
                        index = index + cmdIndex + 1
                        cmdIndex = 0
                        while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER:
                            cmdIndex = cmdIndex + 1
                        posRoby= int(packet[index:index+cmdIndex])
                        cmdIndex = 0
                        commandReceived = False
                    elif (cmdBuffer == "IMU"): #Check if the received string is "IMU"
                        while packet[index+cmdIndex] != VALUE_SEP:
                            cmdIndex = cmdIndex + 1
                        velRobx= int(packet[index:index+cmdIndex])
                        index = index + cmdIndex + 1
                        cmdIndex = 0
                        while packet[index+cmdIndex] != VALUE_SEP:
                            cmdIndex = cmdIndex + 1
                        velRoby= int(packet[index:index+cmdIndex])
                        index = index + cmdIndex + 1
                        cmdIndex = 0
                        while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER:
                            cmdIndex = cmdIndex + 1
                        posRobt= int(packet[index:index+cmdIndex])
                        cmdIndex = 0
                        commandReceived = False
                    elif (cmdBuffer == "OPP"): #Check if the received string is "OPP" #which indicates tracking the position of opponent
                        while packet[index+cmdIndex] != VALUE_SEP:
                            cmdIndex = cmdIndex + 1
                        posOppx= int(packet[index:index+cmdIndex])
                        index = index + cmdIndex + 1
                        cmdIndex = 0
                        while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER:
                            cmdIndex = cmdIndex + 1
                        posOppy= int(packet[index:index+cmdIndex])
                        cmdIndex = 0
                        commandReceived = False
                    elif (cmdBuffer == "BAL"): #Check if the received string is "BAL" #tracking position of the ball
                        while packet[index+cmdIndex] != VALUE_SEP:
                            cmdIndex = cmdIndex + 1
                        posBallx= int(packet[index:index+cmdIndex])
                        index = index + cmdIndex + 1
                        cmdIndex = 0
                        while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER:
                            cmdIndex = cmdIndex + 1
                        posBally= int(packet[index:index+cmdIndex])
                        cmdIndex = 0
                        commandReceived = False
                    else:
                        print("Unknown command")
                        print(cmdBuffer)
                        commandReceived = False

def isWhiteSpace(character):
    if (character == ' '):
        return True
    if (character == '\r'):
        return True
    if (character == '\n'):
        return True
    return False

def push(motorSpeedAbs,inA1,inA2): #pushes data TO the arduino from the pi

    #<MO0-rpm-in1-in2|MO1-rpm-in1-in2|MO2-rpm-in1-in2|MO3-rpm-in1-in2|>
    motor = ""
    outA1 = ""
    outA2 = ""
    for i in range(4):
        motor = motor + str(motorSpeedAbs[i])
        outA1 = outA1 + str(int(inA1[i]==True))
        outA2 = outA2 + str(int(inA2[i]==True))
        if i < 3:
            motor = motor + "-"
            outA1 = outA1 + "-"
            outA2 = outA2 + "-"
        
        packet = "<MOT|" + motor + "-" + outA1 + "-" + outA2 + ">"
    
    ser.write(packet.encode('utf-8'))
    print(packet.encode('utf-8'))

speed = np.array([255, 216, 160, 122])  #(TEST VALUES) indicates each of the four motor speeds in order
ina1 = [False, True, False,  True]  #values of the first gate of each motor driver
ina2 = [True, False,  True, False] #values of the second gate of each motor driver
push(speed,ina1,ina2)
while True:
    # print(posRobx)
    # print(posRoby)
    pull()