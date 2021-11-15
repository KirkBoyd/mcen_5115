import numpy as np
from numpy.random import randn
import time
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
import cv2 as cv
import serial
## Testing git
## Created by Thomas Gira Oct 13, 2021

ser = serial.Serial('COM5',9600) #Windows serial port

## World Frame
#All units are in mm
#The map is 8'x12'
#The map is 2440mm x 3660mm
#x is width y is height

##Initialize Positions
posBallx = 1500
posBally = 2000
posOppx = 500.0
posOppy = 3200
posRobx = 700
posRoby = 150
posRobt = 0
posTargetx = 2440/2
posTargety = 3660
posProtectx = 2440/2 #Our goal
posProtecty = 0 #Our Goal
objective = []
robotMotorSpeed = np.empty((1,4),float) # array to hold speeds for each of four motors
robotVelocity = np.empty((3,1),int) # array to hold velocities?
defense = False # boolean for defense / offense mode

## Robot Parameters
r = 97/2 # radius in mm
ly = 100 # Distance from center of robot to center of wheel in y direction
lx = 100 # Distance from center of robot to center of wheel in x direction
maxRPM = 1000 # Safety value to keep from over running motors

## Plotting
mapScale = .1 #1px = 1cm
MAP = cv.imread("python\Map.PNG") #Read in picture of map
width = int(8*305*mapScale)
height = int(12*305*mapScale)
dimensions = (width,height)
MAP = cv.resize(MAP,dimensions, interpolation=cv.INTER_LINEAR)
## Classes
class PosSensor(object):
    def __init__(self, pos=(0, 0), noise_std=1.):
        self.noise_std = noise_std
        self.pos = [pos[0], pos[1]]
        
    def read(self):
        self.pos[0]
        self.pos[1]
        
        return [self.pos[0] + randn() * self.noise_std,
                self.pos[1] + randn() * self.noise_std]
## Functions
def motorSpeed(V): #Input vector (vx,vy,theta) [mm/s],[mm/s],[rad/s]
    global robotMotorSpeed
    T = np.array([[1,-1,-lx-ly],[1,1,lx+ly],[1,1,-lx-ly],[1,-1,lx+ly]])/r #Translation matrix
    motorSpeed = np.matmul(T,V) #Forward Kinematics
    motorSpeed = motorSpeed*maxRPM/max(abs(motorSpeed)) #Normalize rpm
    robotMotorSpeed = motorSpeed
    motorSpeed = [motorSpeed[0][0],motorSpeed[1][0],motorSpeed[2][0],motorSpeed[3][0]]
    motorSpeedNorm = np.interp(motorSpeed,[-1000, 1000],[-255,255]) #Normalize rpm to 255 maximum value
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
    T = np.array([[-np.cos(posRobt),np.sin(posRobt),0],[np.sin(posRobt),np.cos(posRobt),0],[0,0,1]]) #Transformation matrix
    delta = np.array([[dX],[dY],[wTheta]]).astype(float)
    return np.matmul(T,delta)
def robot2World(cords): #Takes in robot coordinates (rx, ry, rt) and returns world coordinates (wx,wy,wt)
    x = posRobx + cords[0]*np.cos(posRobt)-cords[1]*np.sin(posRobt)
    y = posRoby + cords[0]*np.sin(posRobt)+cords[1]*np.cos(posRobt)

    return [x,y]
def goal2Speed(goal,bias): #Function to get robot velocity based off the curr robot position and goal positions, bias is for turning speed
    robotGoalCords = world2Robot(goal)
    vx = robotGoalCords[0]
    vy = robotGoalCords[1]
    vt = (goal[2] - posRobt)*bias
    
    return [vx,vy,vt]
def updateVelocity(): #Update the global velocity of the robot for positioning data
    global robotVelocity
    T = np.array([[1,1,1,1],[-1,1,1,-1],[-1/(lx+ly),1/(lx+ly),-1/(lx+ly),1/(lx+ly)]])/r #Translation matrix
    tempVelocity = np.dot(T,robotMotorSpeed) #Inverse Kinematics
    vx = tempVelocity[0]*np.cos(posRobt)-tempVelocity[1]*np.sin(posRobt)
    vy = tempVelocity[0]*np.sin(posRobt)+tempVelocity[1]*np.cos(posRobt)
    vt = tempVelocity[2]
    robotVelocity = [-vx[0],vy[0],vt[0]] #THIS SHIT IS FUCKED (IDK WHY ITS -vx)
    pass
def updatePosRob(): #Updates the position based on the global robot velocity values
    global posRobx
    global posRoby
    global posRobt

    updateVelocity()
    vScale = 1/10
    posRobx = posRobx + robotVelocity[0]*vScale
    posRoby = posRoby + robotVelocity[1]*vScale
    posRobt = posRobt + robotVelocity[2]*vScale
    pass
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
    motorSpeedAbs = data[0]
    inA1 = data[1]
    inA2 = data[2]
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
def kalman(H,Q,R,dt):
    F = np.array([[1, dt, 0, 0,],   #x    Design State Transformation Matrix
                  [0, 0, 1, dt],    #y
                  [0, 1, 0, 0]      #vx
                  [0, 0, 0, 1]])    #vy

## Main Loop
test = 1
try:
    while test != 5:
        posOppy = posOppy - 1
        defense = posBally < 3660/2
        if defense: #Ball is on our half
            if checkDefensive(): #We are in a protective position
                objective = scoringPosition() #Try to score
                if positionCheck(objective): #Robot is touching ball
                    push(motorSpeed((goal2Speed((posTargetx,posTargety,objective[2]),10))))
                    updatePosRob()
                    updateBall()
                else: #Robot is far from ball
                    push(motorSpeed((goal2Speed(objective,10))))
                    updatePosRob()
            else: #We are not in a protective position
                objective = defensePosition()
                push(motorSpeed((goal2Speed(objective,0))))
                updatePosRob()
        else: #Ball is on their half
            objective = scoringPosition()
            if positionCheck(objective): #Robot is touching ball
                push(motorSpeed((goal2Speed((posTargetx,posTargety,objective[2]),10))))
                updatePosRob()
                updateBall()
            else: #Robot is far from ball
                push(motorSpeed((goal2Speed(objective,5))))
                updatePosRob()
        updateMap()
        #test = test + 1
        if cv.waitKey(20) & 0xFF==ord('d'):
            break
        #time.sleep(100)
except KeyboardInterrupt:
    print("turds")
    cv.destroyAllWindows()