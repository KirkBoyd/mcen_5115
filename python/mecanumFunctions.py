import numpy as np

def motorSpeed(V): #Input vector (vx,vy,theta) [mm/s],[mm/s],[rad/s]
    r = 97/2 # radius in mm
    ly = 100 # Distance from center of robot to center of wheel in y direction
    lx = 100 # Distance from center of robot to center of wheel in x direction
    maxRPM = 1000

    T = np.array([[1,-1,-lx-ly],[1,1,lx+ly],[1,1,-lx-ly],[1,-1,lx+ly]])/r
    # print(T)
    # print(V)
    motorSpeed = np.dot(T,V) #Forward Kinematics

    if max(abs(motorSpeed)) > 1000: #Motor exceeds maximum rpm
        motorSpeed = motorSpeed*1000/max(abs(motorSpeed)) #Normalize rpm
    direction = motorSpeed >0 #True if forward

    motorSpeedNorm = np.interp(motorSpeed,[-1000, 1000],[-255,255]) #Normalize rpm to 255 maximum value
    motorSpeedOut = motorSpeedNorm.astype('int32') #Convert motor rpm to integer

    # print(motorSpeed)
    # print(motorSpeedNorm)
    # print(motorSpeedOut)

    inA1 = motorSpeedNorm >= 0
    inA2 = motorSpeedNorm <= 0

    # print(inA1)
    # print(inA2)

    motorSpeedAbs = abs(motorSpeedOut)
    # print(motorSpeedAbs)
    return motorSpeedAbs,inA1,inA2

#Test Cases
forward = np.array([[5000],[0],[0]])
backward = np.array([[-5000],[0],[0]])
forward_left = np.array([[5000],[5000],[0]])
# print('Forward')
# motorSpeed(forward)
# print('backward')
# motorSpeed(backward)
print('forward_left')
motorSpeed(forward_left)