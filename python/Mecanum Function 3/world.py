import numpy as np
import serial

#from python.mecanumFunctions import isWhiteSpace, motorSpeed, reamonToWorldX

class opponentClass:
    def __init__(self,color = "blue"):
        self.x = 0
        self.y = 0
        self.color = color

class robotClass:
    def __init__(self,color = "green",bias = 0,biasReceived=False):
        #Robot Parameters
        self.r  = 97/2
        self.lx = 125
        self.ly = 90
        
        #Positional Data
        self.x = 0
        self.y = 0
        self.theta = 0
        
        #Goal Positions
        self.goalX = 310
        self.goalY = 128
        self.goalTheta = 0
        
        #Goal Velocities
        self.goalVelocityX = 0
        self.goalVelocityY = 0
        self.goalVelocityTheta = 0
        
        #Motor speeds and directions
        self.speeds = np.array([0,0,0,0])
        self.directions = np.array([0,0,0,0])
        
        #Imu Bias
        self.imuBias = bias
        self.imuBiasRecieved = biasReceived
        
        #Game Variables
        self.color = color

class ballClass:
    def __init__(self,color = "yellow"):
        self.x = 0
        self.y = 0
        self.color = color

class worldClass():
    def __init__(self,team,opponentColor,posTargetx,posTargety,posProtectx,posProtecty,bias = 0,biasReceived=False):
        self.opponent = opponentClass(opponentColor)
        self.robot = robotClass(team,bias,biasReceived)
        self.ball = ballClass('yellow')
        self.radioData = np.zeros(6)
        
        #Target goal location
        self.posTargetX = posTargetx
        self.posTargetY = posTargety

        #Our goal location
        self.posProtectX = posProtectx
        self.posProtectY = posProtecty

        #Scoring position location
        self.posScoringX = 180
        self.posScoringY = 250
        if team == 'green':
            self.posScoringTheta = 0
        elif team == 'blue':
            self.posScoringTheta = 180
        else:
            print("Unable to set team")
        
        print("----------------")
        print("Game Initialized")
        print("----------------")

    def world2robot(self,wx,wy): #Function to convert world to robot coordinates (Complete)
        dx = wx - self.robot.x
        dy = wy - self.robot.y
        
        robotx = dy*np.cos(self.robot.theta) +dx*np.sin(self.robot.theta)
        roboty = dy*np.sin(self.robot.theta) + dx*np.cos(self.robot.theta)

        rtheta = np.arctan2(roboty,robotx)
        return robotx, roboty, rtheta

    def reamon2world(self,reamonx,reamony): #Function to convert camera coordinates to world coordinates (Complete)
        worldx = 375 - reamony
        worldy = 500 - reamonx
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
                    angle = angle - self.robot.imuBias
                    if angle > np.pi:
                        angle = angle - 2*np.pi
                        
                    self.robot.theta = angle
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
        for index in range(3):
            #print("coord Color:" + colorList[index] + " Robot Color: " + self.robot.color)
            if self.robot.color == colorList[index]:
                worldPositions = self.reamon2world(self.radioData[index*2],self.radioData[index*2+1])
                self.robot.x = worldPositions[0]
                self.robot.y = worldPositions[1]
                continue
            if self.opponent.color == colorList[index]:
                worldPositions = self.reamon2world(self.radioData[index*2],self.radioData[index*2+1])
                self.opponent.x = worldPositions[0]
                self.opponent.y = worldPositions[1]
                continue
            if self.ball.color == colorList[index]:
                worldPositions = self.reamon2world(self.radioData[index*2],self.radioData[index*2+1])
                self.ball.x = worldPositions[0]
                self.ball.y = worldPositions[1]
                continue