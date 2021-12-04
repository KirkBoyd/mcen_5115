import numpy as np
import serial

#from python.mecanumFunctions import isWhiteSpace, motorSpeed, reamonToWorldX

class opponentClass:
    def __init__(self,color = "blue"):
        self.x = 0
        self.y = 0
        self.color = color

class robotClass:
    def __init__(self,color = "green"):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.speeds = np.array([0,0,0,0])
        self.directions = np.array([0,0,0,0])
        self.imuBias = 0
        self.imuBiasRecieved = False
        self.color = color

class ballClass:
    def __init(self,color = "yellow"):
        self.x = 0
        self.y = 0
        self.color = color

class worldClass():
    def __init__(self):
        self.opponent = opponentClass()
        self.robot = robotClass()
        self.ball = ballClass()
        self.radioData = np.zeros(6)
        print("----------------")
        print("Game Initialized")
        print("----------------")

    def world2robot(self,wx,wy): #Function to convert world to robot coordinates (Complete)
        robotx = self.robot.x + wx*np.cos(self.robot.theta) - wy*np.sin(self.robot.theta)
        roboty = self.robot.y + wx*np.sin(self.robot.theta) + wy*np.cos(self.robot.theta)

        rtheta = np.arctan2(roboty,robotx)
        return robotx, roboty, rtheta

    def reamon2world(self,reamonx,reamony): #Function to convert camera coordinates to world coordinates (Complete)
        worldx = 500 - reamony
        worldy = 375 - reamonx
        return worldx,worldy

    def robot2world(): #Function to convert world coordinates to robot coordinates (Incomplete)
        pass

    def isWhiteSpace(self,character): #checks if input value from serial is a blank / whitespace character
        if (character == ' '):
            return True
        if (character == '\r'):
            return True
        if (character == '\n'):
            return True
        if (character == '⸮'):
            return True
        return False

    def parseImu(self,inPacket):
        START_MARKER = '<'  #marks the beginning of a data packet
        END_MARKER = '>'    #marks the end of a data packet
        VALUE_SEP = '-'     #?? *Kirk stop being an idiot*
        receiving = False   #set to true when start marker is received, set to false when end marker is received
        index = 0
        lenInPacket = len(inPacket)
        while index < lenInPacket: #Loops through incoming packet
            serialByte = inPacket[index] #Grabs a character
            if self.isWhiteSpace(serialByte): # Cancel everything if their is a bad character
                return
            if serialByte == START_MARKER and not receiving:
                receiving = True
                index += 1
                continue
            if serialByte == START_MARKER and receiving: #Bad data
                return
            if receiving: #Look for packet
                if serialByte != END_MARKER:
                    try:
                        float(inPacket[index])
                    except ValueError:
                        return
                    index += 1
                if serialByte == END_MARKER:
                    try:
                        angle = (float(inPacket[1:index])*2)*(np.pi/180)
                        
                    except ValueError:
                        return
                    if not self.robot.imuBiasRecieved:
                        self.robot.imuBias = angle
                        self.robot.imuBiasRecieved = True
                    self.robot.theta = angle - self.robot.imuBias
                    return
        
    def parseRadio(self,inPacket):
        START_MARKER = '<'  #marks the beginning of a data packet
        END_MARKER = '>'    #marks the end of a data packet
        VALUE_SEP = '-'     #?? *Kirk stop being an idiot*
        receiving = False   #set to true when start marker is received, set to false when end marker is received
        index = 0
        radioDataIndex = 0
        valueString = ""
        lenInPacket = len(inPacket)
        while index < lenInPacket-1: #Loops through incoming packet
            serialByte = inPacket[index] #Grabs a character
            if self.isWhiteSpace(serialByte): # Cancel everything if their is a bad character
                return
            if serialByte == START_MARKER and not receiving:
                receiving = True
                index += 1
                continue
            if serialByte == START_MARKER and receiving: #Bad data
                return
            if receiving: #Look for packet
                if serialByte != VALUE_SEP and serialByte != END_MARKER:
                    try:
                        float(serialByte)
                        valueString = valueString + serialByte
                        index +=1
                    except ValueError:
                        print("Error Converting: "  + serialByte + " Character To Float in parseRadio")
                        return
                elif serialByte == VALUE_SEP:
                    try:
                        value = int(valueString)
                        self.radioData[radioDataIndex] = value
                        valueString = ""
                        index += 1
                        radioDataIndex += 1
                    except ValueError:
                        print("Error Converting Character To Float in parseRadio")
                        return
                elif serialByte == END_MARKER:
                    try:
                        value = int(valueString)
                        self.radioData[radioDataIndex] = value
                        self.setBallLocations()
                        return
                    except ValueError:
                        print("Error Converting Character To Float in parseRadio")
                        return
            else:return #Didnt hit anything

    def setBallLocations(self):
        #Radio send data in the order (Green,Blue,Yellow)
        colorList = ["green","blue","yellow"]
        colorIndex = 0
        for index in range(3):
            if self.robot.color == colorList[colorIndex]:
                worldPositions = self.reamon2world(self.radioData[index*2],self.radioData[index*2+1])
                self.robot.x = worldPositions[0]
                self.robot.y = worldPositions[1]
                return
            if self.opponent.color == colorList[colorIndex]:
                worldPositions = self.reamon2world(self.radioData[index*2],self.radioData[index*2+1])
                self.opponent.x = worldPositions[0]
                self.opponent.y = worldPositions[1]
                return
            if self.ball.color == colorList[colorIndex]:
                worldPositions = self.reamon2world(self.radioData[index*2],self.radioData[index*2+1])
                self.ball.x = worldPositions[0]
                self.ball.y = worldPositions[1]
                return
            colorIndex += 1