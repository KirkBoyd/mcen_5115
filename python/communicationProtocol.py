import numpy as np
import serial

ser = serial.Serial('COM3',9600) #Windows
#ser = serial.Serial('\dev\ttyUSB*',9600) #Unix

#List of Commands or functions
    #Input
        #Radio
            #Robot
            #Enemy
            #Ball
        #Pixy
            #Obj
            #Size
            #X
            #Y
        #IMU
            #Vx
            #Vy
            #THETAz
    #Output
        #Motors
            #4 Motors MT0 MT1 MT2 MT3
            #rpm 0:999
            #inA1
            #inA2

START_MARKER = '<'
END_MARKER = '>'
COMMAND_SEP = '|'
VALUE_SEP = ','

receiving = False   #set to true when start marker is received, set to false when end marker is received
commandReceived = False    #set to true when command separator is received (or if command buffer is full)

def parse():
    if (ser.in_waiting > 0):
        packet = ser.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
        print(packet[0])


while True:
    parse()