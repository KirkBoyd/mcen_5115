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

ser = serial.Serial('COM4',9600) #Windows
#ser = serial.Serial('\dev\ttyUSB*',9600) #Unix

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


def pull():
    global posRobx
    global posRoby
    global posRobt
    global posOppx
    global posOppy
    global posBallx
    global posBally


    START_MARKER = '<'
    END_MARKER = '>'
    COMMAND_SEP = '|'
    VALUE_SEP = '-'

    receiving = False   #set to true when start marker is received, set to false when end marker is received
    commandReceived = False    #set to true when command separator is received (or if command buffer is full)

    cmdLen = 3 #length of command to recieve
    cmdBuffer = ""

    index = 0
    cmdIndex = 0
    if (ser.in_waiting > 0):
        packet = ser.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
        print(packet)
        if (isWhiteSpace(packet)):
            return # Ignore whitespace
        while index < len(packet):
            serialByte = packet[index]
            # print(serialByte)
            if (isWhiteSpace(serialByte)):
                return
            if serialByte == START_MARKER:
                receiving = True
                commandReceived = True
                index = index + 1
                continue
            if(receiving):
                serialByte = packet[index]
                if not commandReceived:
                    if (serialByte == COMMAND_SEP): #If the command separator is received
                        index = index + 1
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
                        print("ROB")
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
                    elif (cmdBuffer == "OPP"): #Check if the received string is "OPP"
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
                    elif (cmdBuffer == "BAL"): #Check if the received string is "OPP"
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

def push(motorSpeedAbs,inA1,inA2):

    #<MO0-rpm-in1-in2|MO1-rpm-in1-in2|MO2-rpm-in1-in2|MO3-rpm-in1-in2|>
    packet = "<"
    motors = ["MO0","MO1","MO2","MO3"]
    for i in range(4):
        packet = packet + motors[i] + "-" + str(motorSpeedAbs[i]) + "-" + str(inA1[i]) + "-" + str(inA2[i])
        if i < 3:
            packet = packet + "|"
        else: packet = packet + ">"
    
    ser.write(packet.encode('utf-8'))
    print(packet.encode('utf-8'))

speed = np.array([255, 216, 160, 122])
ina1 = [False, True, False,  True]
ina2 = [True, False,  True, False]
push(speed,ina1,ina2)
while True:
    # print(posRobx)
    # print(posRoby)
    pull()