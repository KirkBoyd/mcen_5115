import numpy as np
import time
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
import cv2 as cv
## Testing git
## Created by Thomas Gira Oct 13, 2021

##Initialize Positions
posBallx = 500.0
posBally = 500.0
posOppx = 500.0
posOppy = 800.0
posRobx = 610
posRoby = 2000
posRobt = .75
robotMotorSpeed = []
robotVelocity = []

## Robot Parameters
r = 97/2 # radius in mm
ly = 100 # Distance from center of robot to center of wheel in y direction
lx = 100 # Distance from center of robot to center of wheel in x direction
maxRPM = 1000

## Plotting
# worldMap = plt.figure("World Map",[2,3])
# ax = plt.gca()
# ax.set_xlim([0, 8*305])
# ax.set_ylim([0,12*305])
mapScale = .1


## Functions
def motorSpeed(V): #Input vector (vx,vy,theta) [mm/s],[mm/s],[rad/s]
    T = np.array([[1,-1,-lx-ly],[1,1,lx+ly],[1,1,-lx-ly],[1,-1,lx+ly]])/r #Translation matrix
    motorSpeed = np.dot(T,V) #Forward Kinematics
    motorSpeed = motorSpeed*maxRPM/max(abs(motorSpeed)) #Normalize rpm
    robotMotorSpeed = motorSpeed #Update global robot speed
    direction = motorSpeed > 0 #True if forward
    motorSpeedNorm = np.interp(motorSpeed,[-1000, 1000],[-255,255]) #Normalize rpm to 255 maximum value
    motorSpeedOut = motorSpeedNorm.astype('int32') #Convert motor rpm to integer
    inA1 = motorSpeedNorm >= 0 #Logic for direction
    inA2 = motorSpeedNorm <= 0 #Logic for direction
    motorSpeedAbs = abs(motorSpeedOut) #Make motor speed a magnitude
    return motorSpeedAbs,inA1,inA2

def world2Robot(cords): #Function to converd world coordinates to robot coordinates
    dX = posRobx-cords[0]
    dY = posRoby-cords[1]
    wTheta = np.arctan2(dY,dX)
    #print(wTheta)
    T = np.array([[np.sin(posRobt),np.cos(posRobt),0],[-np.sin(posRobt),np.cos(posRobt),0],[0,0,1]]) #Transformation matrix
    delta = np.array([[dX],[dY],[wTheta]]).astype(float)
    return np.dot(T,delta)

def robot2World(cords):
    x = posRobx + cords[0]*np.cos(posRobt)-cords[1]*np.sin(posRobt)
    y = posRoby + cords[0]*np.sin(posRobt)+cords[1]*np.cos(posRobt)

    return [x,y]

def goal2Speed(goal): #Function to get robot velocity based off the curr robot position and goal positions
    robotGoaCords = world2Robot(goal)
    V = []
    V[0] = goal[0] - posRobx
    V[1] = goal[1] - posRoby
    V[2] = goal[2] - posRobt
    
    return V

def updateVelocity(): #Update the global velocity of the robot por positioning data
    T = np.array([[1,1,1,1],[-1,1,1,-1],[-1/(lx+ly),1/(lx+ly),-1/(lx+ly),1/(lx+ly),],[1,-1,lx+ly]])/r #Translation matrix
    robotVelocity = np.dot(T,robotMotorSpeed) #Forward Kinematics

def robotTriangle():
    L = 250
    p1 = (L*np.sqrt(3)/3,0)
    p2 = (-L*np.sqrt(3)/6,L/3)
    p3 = (-L*np.sqrt(3)/6,-L/3)

    w1 = robot2World(p1)
    w2 = robot2World(p2)
    w3 = robot2World(p3)

    pts = np.array([(int(w1[0]*mapScale),int(w1[1]*mapScale)),(int(w2[0]*mapScale),int(w2[1]*mapScale)),(int(w3[0]*mapScale),int(w3[1]*mapScale))])
    return pts

def updateMap():
    map = cv.imread("python\Map.PNG")
    width = int(8*305*mapScale)
    height = int(12*305*mapScale)
    dimensions = (width,height)
    map = cv.resize(map,dimensions, interpolation=cv.INTER_LINEAR)
    # Blue for robot
    pts = robotTriangle()
    cv.drawContours(map,[pts],0,(0,255,0),-1)
    #Yellow for opponent
    cv.circle(map, (int(posOppx*mapScale),int(posOppy*mapScale)), 10, (255,0,255), thickness=-1)
    #Red for ball
    cv.circle(map, (int(posBallx*mapScale),int(posBally*mapScale)), 10, (255,0,0), thickness=-1)

    cv.imshow('Map',map)
    pass

#Test Cases
# forward = np.array([[5000],[0],[0]])
# backward = np.array([[-5000],[0],[0]])
# forward_left = np.array([[5000],[5000],[0]])
# print('Forward')
# motorSpeed(forward)
# print('backward')
# motorSpeed(backward)
# print('forward_left')
# motorSpeed(forward_left)

#print(world2Robot(13,5,9,9,np.pi/4))



## Main Loop
try:
    while True:
        posOppy = posOppy + 1
        updateMap()
        if cv.waitKey(20) & 0xFF==ord('d'):
            break
except KeyboardInterrupt:
    print("turds")
    cv.destroyAllWindows()