import numpy as np
import time
#import cv2 as cv
from numpy.core.numeric import ones
import serial
import signal

from serial.serialutil import SerialTimeoutException
## Created by Thomas Gira Oct 13, 2021
# Last edited by Kirk Boyd Nov 26, 21

#ser = serial.Serial('COM6',4800) #winndows serial port
ser0 = serial.Serial('/dev/ttyACM0',9600,write_timeout=.5,timeout=.5) #IMU and Motors
ser1 = serial.Serial('/dev/ttyACM1',9600,write_timeout=.5,timeout=.5) #Radio
time.sleep(1)

ser0.flush()
ser0.reset_output_buffer()
ser0.reset_input_buffer()

ser1.flush()
ser1.reset_output_buffer()
ser1.reset_input_buffer()
connected = True
## World Frame
#All units are in mm
#The map is 8'x12'
#The map is 2440mm x 3660mm
#x is width y is height

# Time Step Stuff
oldTime = time.time()
newTime = time.time()
dt = newTime-oldTime

##Initialize Positions
posBallx = 1220
posBally = 2000
posOppx = 500.0
posOppy = 3200
posRobx = 2440/4
posRoby =3660/6
posRobt = np.pi/2
posTargetx = 192
posTargety = 499
posProtectx = 183 #Our goal
posProtecty = -26 #Our Goal
objective = []
robotMotorSpeed = np.empty((1,4),float)
robotVelocity = np.empty((3,1),int)
defense = False
objective
imuBias = 0#Correct for gps angle
imuBiasReceived = False
## World Calibration
# North is towardsglass
# East is towards Lockers
# NW actual corner:
# 
xmin = 5
xmax = 350
ymin = 25
ymax = 495


def printInfo():
    print("posBallX: "+ str(posBallx) +" posBallY: "+ str(posBally) +" posOPPX: "+ str(posOppx) +" posOppY: "+ str(posOppy) +" posRobX: "+ str(posRobx) +" posRoby: "+ str(posRoby) + " posRobt: " + str(posRobt))
    print("Vx: "+ str(robotVelocity[0]) + " Vy: " + str(robotVelocity[1])+ " Vt: " + str(robotVelocity[2]))
    #print("Objective: " + str(objective))
    
# Robot Parameters
# Full Scale
r = 97/2 # radius in mm
ly = 90 # Distance from center of robot to center of wheel in y direction
lx = 125 # Distance from center of robot to center of wheel in x direction
maxRPM = 150*0.104719755

# ## Test Bot
# r = 33
# ly = 69
# lx = 62
#maxRPM = 5#Actually in rad/s Dont want to update all vars

# Plotting
# mapScale = .1 #1px = 1cm
# MAP = cv.imread("python\Map.PNG") #Read in picture of map
# width = int(8*305*mapScale)
# height = int(12*305*mapScale)
# dimensions = (width,height)
# MAP = cv.resize(MAP,dimensions, interpolation=cv.INTER_LINEAR)

def motorSpeed(V): #Input vector (vx,vy,theta) [mm/s],[mm/s],[rad/s]
    global robotMotorSpeed
    #print(V)
    T = np.array([[1,-1,-lx-ly],
                  [1,1,lx+ly],
                  [1,1,-lx-ly],
                  [1,-1,lx+ly]])/r #Translation matrix
    motorSpeed = np.matmul(T,V) #Forward Kinematics
    motorSpeed = motorSpeed*maxRPM/max(abs(motorSpeed)) #Normalize rpm
    robotMotorSpeed = motorSpeed
    motorSpeed = [motorSpeed[0],motorSpeed[1],motorSpeed[2],motorSpeed[3]]
    motorSpeedNorm = np.interp(motorSpeed,[-maxRPM, maxRPM],[-254,254]) #Normalize rpm to 255 maximum value
    motorSpeedOut = motorSpeedNorm.astype('int32') #Convert motor rpm to integer
    inA1 = motorSpeedNorm >= 0 #Logic for direction
    inA2 = motorSpeedNorm <= 0 #Logic for direction
    motorSpeedAbs = abs(motorSpeedOut) #Make motor speed a magnitude
    return motorSpeedAbs,inA1,inA2


def world2Robot(cords): #Function to convert world coordinates to robot coordinates
    dX = cords[0] - posRobx
    dY = cords[1] -posRoby
    wTheta = np.arctan2(dY,dX)
    #print(wTheta)
    T = np.array([[-np.cos(posRobt),np.sin(posRobt),0],
                  [np.sin(posRobt),np.cos(posRobt),0],
                  [0,0,1]]) #Transformation matrix
    delta = np.array([[dX],[dY],[wTheta]]).astype(float)
    return np.matmul(T,delta)

def robot2World(cords): #Takes in robot coordinates (rx, ry, rt) and returns world coordinates (wx,wy,wt)
    x = posRobx + cords[0]*np.cos(posRobt)-cords[1]*np.sin(posRobt)
    y = posRoby + cords[0]*np.sin(posRobt)+cords[1]*np.cos(posRobt)

    return [x,y]

def goal2Speed(goal,bias,delta=np.pi/6): #Function to get robot velocity based off the curr robot position and goal positions, bias is for turning speed
    global robotVelocity
    
    
    robotGoalCords = world2Robot(goal)
    dy = robotGoalCords[0]
    dx = robotGoalCords[1]
    dt = goal[2] - posRobt
    if abs(dt) > np.pi:
        dt = -dt
    if abs(dt) < delta:
        dt = 0
    dt = dt*bias
    #print("goal2Speed: "+str(dx[0])+str(dy[0]) +str(dt))
    robotVelocity = [dx[0],dy[0],dt]
    return [dx[0],dy[0],dt]

def updateVelocity(): #Update the global velocity of the robot for positioning data
    global robotVelocity
    T = r/4*np.array([[1,1,1,1],
                      [-1,1,1,-1],
                      [-1/(lx+ly),1/(lx+ly),-1/(lx+ly),1/(lx+ly)]]) #Translation matrix
    tempVelocity = np.dot(T,robotMotorSpeed) #Inverse Kinematics
    #print(tempVelocity)
    #print(posRobt)
    vx = tempVelocity[0]*np.cos(posRobt)-tempVelocity[1]*np.sin(posRobt)
    vy = tempVelocity[0]*np.sin(posRobt)+tempVelocity[1]*np.cos(posRobt)
    vt = tempVelocity[2]
    robotVelocity = [vx,vy,vt]
    pass

def updatePosRob(): #Updates the position based on the global robot velocity values
    global posRobx
    global posRoby
    global posRobt

    updateVelocity()
    getTime()
    #print(dt)
    #print(robotVelocity)
    #print(posRobx)
    posRobx = posRobx + robotVelocity[0]*dt
    posRoby = posRoby + robotVelocity[1]*dt
    #posRobt = posRobt + robotVelocity[2]*dt
    pass

def getTime():
    global oldTime
    global newTime
    global dt

    newTime = time.time()
    dt = newTime - oldTime
    oldTime = newTime
    
def robotTriangle(): #Returns the points of a triangle corresponding to the robot's position and orientation.
    L = 250
    p1 = (L*np.sqrt(3)/3,0)
    p2 = (-L*np.sqrt(3)/6,L/3)
    p3 = (-L*np.sqrt(3)/6,-L/3)

    w1 = robot2World(p1)
    w2 = robot2World(p2)
    w3 = robot2World(p3)

    pts = np.array([(int(w1[0]*mapScale),height - int(w1[1]*mapScale)),(int(w2[0]*mapScale),height - int(w2[1]*mapScale)),(int(w3[0]*mapScale),height - int(w3[1]*mapScale))])
    return pts

def updateMap(): #Updates minimap
    map = MAP.copy()
    # Blue for robot
    pts = robotTriangle()
    cv.drawContours(map,[pts],0,(255,0,0),-1)
    #Yellow for opponent
    cv.circle(map, (int(posOppx*mapScale),height - int(posOppy*mapScale)), 10, (0,255,255), thickness=-1)
    #Red for ball
    cv.circle(map, (int(posBallx*mapScale),height - int(posBally*mapScale)), 10, (0,0,255), thickness=-1)
    #X for goal
    cv.drawMarker(map, (int(objective[0]*mapScale),height-int(objective[1]*mapScale)) ,(0,255,0), markerType=cv.MARKER_TILTED_CROSS , markerSize=5, thickness=2, line_type=cv.LINE_AA)

    cv.imshow('Map',map)
    pass

def scoringPosition():  #Retruns a position and orientation of the robot that is in line with the ball and goal
    theta = np.arctan2(posTargety - posBally, posTargetx - posBallx)
    x = posBallx - 150*np.cos(theta)
    y = posBally - 150*np.sin(theta)
    return (x,y,theta)

def positionCheck(position): #Checks if the robot is close enough to its desired position
    distance = 100 #Maximum distance
    angle = np.pi/16 #Maximum angle
    return abs(posRobt-position[2]) < angle and np.sqrt((posRobx - position[0])**2 + (posRoby-position[1])**2) < distance

def updateBall(): #Updates the position of the ball to infront of the robot
    global posBallx
    global posBally

    posBallx = posRobx + 150*np.cos(posRobt)
    posBally = posRoby + 150*np.sin(posRobt)

def defensePosition(): #Returns the point which is closest to the robot on a line from the ball to our goal
    if posBally > posRoby: #Check that the robot is at least goal side
        m = np.array([posBallx-posProtectx,posBally-posProtecty]) #Direction vector of line
        A = np.array([posProtectx,posProtecty]) #Origin of line
        P = np.array([posRobx,posRoby]) #Robot point
        PA = P-A #From goal to rob

        pose = np.dot(PA,m)/np.linalg.norm(m)**2*m + A
        point = (pose[0],pose[1],np.arctan2(pose[0],pose[1]))
    else: point = (posProtectx,posBally,np.pi/2) #go to goal, Can add more functionality
    return point

def checkDefensive(): #Checks if the robot is close enough to the line from the ball to our goal
    m = np.array([posBally-posProtecty,posProtectx-posBallx]) #Direction vector of line
    A = np.array([posProtectx,posProtecty]) #Origin of line
    P = np.array([posRobx,posRoby]) #Robot point
    PA = A-P #From goal to rob
    d = np.dot(m/np.linalg.norm(m),PA)

    return d<50

def push(data): #pushes data TO the arduino from the pi
    if ser0.out_waiting < 10:
        #print("Pushing: " + str(data))
        motorSpeedAbs = data[0]
        inA1 = data[1]
        inA2 = data[2]
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
                
        packet = "<MOT|" + motor + "-" + outA1 + "-" + outA2 + ">\n"
        if connected:
            try:
                ser0.write(packet.encode('utf-8'))
                print('Sent:' + packet)
            except serial.serialutil.SerialTimeoutException:
                ser0.reset_output_buffer
                ser1.reset_output_buffer
                return
    else: ser0.reset_output_buffer

def pull():
    if (ser0.in_waiting > 0): 
        #print("Bits")
        try:
            packet = ser0.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
            parse(packet)
        except UnicodeDecodeError:
            print("Invalid Packet")
            return
        #print("Packet: " + packet) #print what was received from the serial port
        #print("Len Packet: " + str(len(packet)))
        if (isWhiteSpace(packet)):
            return # Ignore whitespace
    
    if (ser1.in_waiting > 0): 
        #print("Bits")
        try:
            packet = ser1.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
            parse(packet)
        except UnicodeDecodeError:
            print("Invalid Packet")
            return
        #print("Packet: " + packet) #print what was received from the serial port
        #print("Len Packet: " + str(len(packet)))
        if (isWhiteSpace(packet)):
            return # Ignore whitespace
def parse(packet): #pulls (or receives) data from the arduino on the pi
    #print("Pulling")
    global posRobx
    global posRoby
    global posRobt
    global posOppx
    global posOppy
    global posBallx
    global posBally
    global imuBias
    global imuBiasReceived


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
    #print(packet)
    lenPacket = len(packet)
    while index < lenPacket: #step through each byte of the packet
        #print("Index: " + str(index))
        serialByte = packet[index] #the byte we are looking at is the one currently at index
        #print(serialByte)
        if (isWhiteSpace(serialByte)): #ignore whitespace again if found
            return
        if serialByte == START_MARKER and not receiving: #if start marker is found
            #print("Start marker received")
            receiving = True #record that a packet is beginning to be received
            commandReceived = False #record that a packet was in fact received
            index = index + 1
            continue
        if serialByte == START_MARKER and receiving: #Reset everything if starting again
            return
        if(receiving): #if looking for a packet
            serialByte = packet[index]
            #print("Serial Byte: " + serialByte)
            if not commandReceived:
                if (serialByte == COMMAND_SEP): #If the command separator is received
                    index = index + 1 #count forward one because there is not a command in this byte
                    commandReceived = True
                    #print("command recieved")
                    continue
                elif (serialByte == END_MARKER): #If end marker is reached
                    return
                else:
                    index = index + 1
            else:
                cmdBuffer = packet[index-4:index-1]
                #print("cmdBuffer: " + cmdBuffer)
                if (cmdBuffer == "ROB"): #Check if the received string is "ROB"
                    #print("ROB") #this is for position of the robot
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
                    if index+cmdIndex >= lenPacket: return
                    while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER:
                        #print("test")
                        try:
                            float(packet[index+cmdIndex])
                            #print("adding: " + packet[index+cmdIndex])
                        except ValueError:
                            print("Invalid packet contents")
                            return
                        cmdIndex = cmdIndex + 1
                        if index+cmdIndex >= lenPacket: return
                    #print("Value: " + packet[index:index+cmdIndex])
                    try:
                        if not imuBiasReceived:
                            imuBias = (float(packet[index:index+cmdIndex])*2)*(np.pi/180)
                            imuBiasReceived = True
                        #print(packet[index:index+cmdIndex])
                        posRobt= (float(packet[index:index+cmdIndex])*2)*(np.pi/180) - imuBias
                        if posRobt > np.pi:
                            posRobt = posRobt-np.pi*2
                    except ValueError:
                        return
                    index = index + cmdIndex + 1
                    cmdIndex = 0
                    commandReceived = False
                    #print("End Function")
                    return
                elif (cmdBuffer == "RAD"): #Check if the received string is "OPP" #which indicates tracking the position of opponent
                    while packet[index+cmdIndex] != VALUE_SEP:
                        if packet[index+cmdIndex] == START_MARKER: return
                        cmdIndex = cmdIndex + 1
                        if index+cmdIndex > lenPacket: return
                    posRoby= int(reamonToWorldX(packet[index:index+cmdIndex]))
                    index = index + cmdIndex + 1
                    cmdIndex = 0
                    while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER and packet[index+cmdIndex] != VALUE_SEP:
                        try:
                            float(packet[index+cmdIndex])
                            #print("adding: " + packet[index+cmdIndex])
                        except ValueError:
                            print("Invalid packet contents")
                            return
                        if packet[index+cmdIndex] == START_MARKER: return
                        cmdIndex = cmdIndex + 1
                        if index+cmdIndex > lenPacket: return
                    posRobx= int(reamonToWorldY(packet[index:index+cmdIndex]))
                    index = index + cmdIndex + 1
                    cmdIndex = 0
                    while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER and packet[index+cmdIndex] != VALUE_SEP:
                        try:
                            float(packet[index+cmdIndex])
                            #print("adding: " + packet[index+cmdIndex])
                        except ValueError:
                            print("Invalid packet contents")
                            return
                        if packet[index+cmdIndex] == START_MARKER: return
                        cmdIndex = cmdIndex + 1
                        if index+cmdIndex >= lenPacket: return
                    posOppy= int(reamonToWorldX(packet[index:index+cmdIndex]))
                    index = index + cmdIndex + 1
                    cmdIndex = 0
                    while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER and packet[index+cmdIndex] != VALUE_SEP:
                        try:
                            float(packet[index+cmdIndex])
                            #print("adding: " + packet[index+cmdIndex])
                        except ValueError:
                            print("Invalid packet contents")
                            return
                        if packet[index+cmdIndex] == START_MARKER: return
                        cmdIndex = cmdIndex + 1
                        if index+cmdIndex >= lenPacket: return
                    posOppx= int(reamonToWorldY(packet[index:index+cmdIndex]))
                    index = index + cmdIndex + 1
                    cmdIndex = 0
                    while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER and packet[index+cmdIndex] != VALUE_SEP:
                        try:
                            float(packet[index+cmdIndex])
                            #print("adding: " + packet[index+cmdIndex])
                        except ValueError:
                            print("Invalid packet contents")
                            return
                        if packet[index+cmdIndex] == START_MARKER: return
                        cmdIndex = cmdIndex + 1
                        if index+cmdIndex >= lenPacket: return
                    posBally= int(reamonToWorldX(packet[index:index+cmdIndex]))
                    index = index + cmdIndex + 1
                    cmdIndex = 0
                    while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER and packet[index+cmdIndex] != VALUE_SEP:
                        try:
                            float(packet[index+cmdIndex])
                            #print("adding: " + packet[index+cmdIndex])
                        except ValueError:
                            print("Invalid packet contents")
                            return
                        if packet[index+cmdIndex] == START_MARKER: return
                        cmdIndex = cmdIndex + 1
                        if index+cmdIndex >= lenPacket: return
                    posBallx= int(reamonToWorldY(packet[index:index+cmdIndex]))
                    index = index + cmdIndex + 1
                    cmdIndex = 0
                    commandReceived = False
                elif (cmdBuffer == "OPP"): #Check if the received string is "OPP" #which indicates tracking the position of opponent
                    while packet[index+cmdIndex] != VALUE_SEP:
                        cmdIndex = cmdIndex + 1
                    posOppy= int(reamonToWorldX(packet[index:index+cmdIndex]))
                    index = index + cmdIndex + 1
                    cmdIndex = 0
                    while packet[index+cmdIndex] != COMMAND_SEP and packet[index+cmdIndex] != END_MARKER:
                        cmdIndex = cmdIndex + 1
                    posOppy= int(reamonToWorldY(packet[index:index+cmdIndex]))
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
        else:
            index = index + 1
    

def isWhiteSpace(character): #checks if input value from serial is a blank / whitespace character
    if (character == ' '):
        return True
    if (character == '\r'):
        return True
    if (character == '\n'):
        return True
    if (character == '⸮'):
        ser1.reset_input_buffer()
        ser1.reset_output_buffer()
        ser0.reset_input_buffer()
        ser0.reset_output_buffer()
        return True
    return False
##end def isWhiteSpace

def reamonToWorldX(x):
    x = int(x)
    return 500 - x

def reamonToWorldY(y):
    y = int(y)
    return 375 -y
    

def main():
    global objective
    test = 1
    try:
        while test != 5:
            pull()
            printInfo()
            defense = posBally < 3660/2
            if defense: #Ball is on our half
                if checkDefensive(): #We are in a protective position
                    objective = scoringPosition() #Try to score
                    if positionCheck(objective): #Robot is touching ball
                        push(motorSpeed((goal2Speed((objective[0],objective[1],objective[2]),10))))
                        #updatePosRob()
                        #updateBall()
                    else: #Robot is far from ball
                        push(motorSpeed((goal2Speed(objective,10))))
                        #updatePosRob()
                else: #We are not in a protective position
                    objective = defensePosition()
                    push(motorSpeed((goal2Speed(objective,0))))
                    #updatePosRob()
            else: #Ball is on their half
                objective = scoringPosition()
                if positionCheck(objective): #Robot is touching ball
                    push(motorSpeed((goal2Speed((objective[0],objective[1],objective[2]),10))))
                    #updatePosRob()
                    #updateBall()
                else: #Robot is far from ball
                    push(motorSpeed((goal2Speed(objective,0))))
                    #updatePosRob()
            # updateMap()
            # #test = test + 1
            # if cv.waitKey(20) & 0xFF==ord('d'):
            #    stop = "<STOP>"
            #    ser.write(stop.encode('utf-8'))
            #    break
            #time.sleep(100)
    except KeyboardInterrupt:
        print("turds")
        stop = "<STOP>"
        ser0.write(stop.encode('utf-8'))
        # cv.destroyAllWindows()
## end def main()


def rotationTest(): # test function to run the pull() function and make sure it is working with serial port
    global objective
    try:
        while True:
            try:
                pull()
            except SerialTimeoutException:
                print("Resetting input buffer")
                ser0.reset_input_buffer()
                ser0.reset_output_buffer()
                ser1.reset_input_buffer()
                ser1.reset_output_buffer()
            try:
                push(motorSpeed((goal2Speed((posRobx,posRoby,0),10))))
                print("push")
            except SerialTimeoutException:
                print("Failure of push")
                ser0.reset_output_buffer()
                print("Push Flushed")
    except KeyboardInterrupt:
        print("turds")
        ser0.write(b"<STP|>")        
## end def rotationTest()        
def coordTest(): # test function to run the pull() function and make sure it is working with serial port
    try:
        while True:
            try:
                pull()
            except SerialTimeoutException:
                print("Resetting input buffer")
                ser0.reset_input_buffer()
                ser0.reset_output_buffer()
                ser1.reset_input_buffer()
                ser1.reset_output_buffer()
            try:
                push(motorSpeed((goal2Speed((183,252,0),.001))))
                printInfo()
            except SerialTimeoutException:
                print("Failure of push")
                ser0.reset_output_buffer()
                print("Push Flushed")
    except KeyboardInterrupt:
        print("turds")
        ser0.write(b"<STP|>")
        
def infoTest():
    try:
        while True:
            pull()
            printInfo()
    except KeyboardInterrupt:
        print("turds")

def pullTest():
    try:
        while True:
            pull()
            print("After Pull")
    except KeyboardInterrupt:
        print("turds")
def test():
    try:
        while True:
            pull()
            vals = motorSpeed(goal2Speed((183,252,0),1))
            push(vals)
            printInfo()
            print(vals)
    except KeyboardInterrupt:
        print("turds")
        ser0.write(b"<STP|>")
test()