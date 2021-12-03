import numpy as np

class opponent:
    def __init__(self):
        self.x = 0
        self.y = 0

class robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

class ball:
    def __init(self):
        self.x = 0
        self.y = 0

class world():
    def __init__(self):
        self.opponent = opponent()
        self.robot = robot()
        self.ball = ball()

    def world2robot(wx,wy):
        robotx = wy
        roboty = wy
        rtheta = 0
        #rtheta = np.atan2(robotx,roboty) Confirm this before adding
        return robotx, roboty, rtheta

    def reamon2world(reamonx,reamony):
        worldx = 500 - reamony
        worldy = 375 - reamonx
        return worldx,worldy

    def robot2world(): #Function to convert world coordinates to robot coordinates
        pass