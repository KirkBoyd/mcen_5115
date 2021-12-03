import numpy as np

class opponentClass:
    def __init__(self):
        self.x = 0
        self.y = 0

class robotClass:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

class ballClass:
    def __init(self):
        self.x = 0
        self.y = 0

class worldClass():
    def __init__(self):
        self.opponent = opponentClass()
        self.robot = robotClass()
        self.ball = ballClass()

    def world2robot(self,wx,wy): #Function to convert world to robot coordinates (Complete)
        robotx = self.robot.x + wx*np.cos(self.robot.theta) - wy*np.sin(self.robot.theta)
        roboty = self.robot.y + wx*np.sin(self.robot.theta) + wy*np.cos(self.robot.theta)

        rtheta = np.arctan2(roboty,robotx)
        return robotx, roboty, rtheta

    def reamon2world(reamonx,reamony): #Function to convert camera coordinates to world coordinates (Complete)
        worldx = 500 - reamony
        worldy = 375 - reamonx
        return worldx,worldy

    def robot2world(): #Function to convert world coordinates to robot coordinates (Incomplete)
        pass