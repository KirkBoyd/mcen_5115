import numpy as np
import time
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
import cv2 as cv
## Testing git
## Created by Thomas Gira Oct 13, 2021

## World Frame
#All units are in mm
#The map is 8'x12'
#The map is 2440mm x 3660mm
#x is width y is height

##Initialize Positions
posBallx = 1500
posBally = 2500
posOppx = 500.0
posOppy = 3200
posRobx = 700
posRoby = 500
posRobt = 0
posTargetx = 2440/2
posTargety = 3660
objective = []
robotMotorSpeed = np.empty((1,4),float)
robotVelocity = np.empty((3,1),int)

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
mapScale = .1 #1px = 1cm
MAP = cv.imread("python\Map.PNG")
width = int(8*305*mapScale)
height = int(12*305*mapScale)
dimensions = (width,height)
MAP = cv.resize(MAP,dimensions, interpolation=cv.INTER_LINEAR)


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

def goal2Speed(goal): #Function to get robot velocity based off the curr robot position and goal positions
    robotGoalCords = world2Robot(goal)
    vx = robotGoalCords[0]
    vy = robotGoalCords[1]
    vt = (goal[2] - posRobt)*10
    
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

def updatePosRob():
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

def updateMap():
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

def scoringPosition():
    theta = np.arctan2(posTargety - posBally, posTargetx - posBallx)
    x = posBallx - 150*np.cos(theta)
    y = posBally - 150*np.sin(theta)
    return (x,y,theta)


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
test = 0
try:
    while test != 5:
        posOppy = posOppy - 1
        objective = scoringPosition()
        motorSpeed((goal2Speed(objective)))
        updatePosRob()
        # print(posRobx,posRoby,posRobt)
        # print(objective)
        # print(robotVelocity)
        updateMap()
        # test = test + 1
        if cv.waitKey(20) & 0xFF==ord('d'):
            break
except KeyboardInterrupt:
    print("turds")
    cv.destroyAllWindows()