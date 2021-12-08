import numpy as np
import os

class robotClass():
    def __init__(self):
        self.r  = 97/2
        self.lx = 125
        self.ly = 90
        self.wheelsSpeeds = np.array([0.0,0.0,0.0,0.0])

        print("-----------------")
        print("Robot Initialized")
        print("-----------------")

    def calculateMotorSpeeds(self,vx,vy,wz):
        self.wheelsSpeeds[0] = (vx-vy-(self.lx+self.ly)*wz)/self.r
        self.wheelsSpeeds[1] = (vx+vy+(self.lx+self.ly)*wz)/self.r
        self.wheelsSpeeds[2] = (vx+vy-(self.lx+self.ly)*wz)/self.r
        self.wheelsSpeeds[3] = (vx-vy+(self.lx+self.ly)*wz)/self.r
        
    def printWheelSpeeds(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        for i in range(4):
            print('%.2f'%self.wheelsSpeeds[2*i],end="")
            print(" ---- ",end="")
            print ('%.2f'%self.wheelsSpeeds[(2*i) + 1])

if __name__ == '__main__':
    robot = robotClass()
    robot.calculateMotorSPeeds(403.0,370.8,10.0)
    robot.printWheelSpeeds()