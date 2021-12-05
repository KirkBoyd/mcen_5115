import numpy as np
def updateGoalPositions(world,goalX,goalY,goalTheta):
    world.robot.goalX = goalX
    world.robot.goalY = goalY
    world.robot.goalTheta = goalTheta
    return world

def defense(world):
    posBallY = world.ball.y
    teamColor= world.robot.color

    midLine = 250

    if teamColor == 'green':
        return posBallY < midLine
    if teamColor == 'blue':
        return posBallY > midLine
    else:
        return True

def checkDefensive(world):
    posBallY = world.ball.y
    posBallX = world.ball.x

    posRobY = world.robot.y
    posRobX = world.robot.x
    posRobTheta = world.robot.theta
    teamColor = world.robot.color

    posProtectX = world.posProtectX
    posProtectY = world.posProtectY

    m = np.array([posBallX - posProtectX,posBallY-posProtectY])
    A = np.array([posProtectX - posProtectY])
    P = np.array([posRobX,posRobY])
    PA = P-A
    
    d = np.dot(m/np.linalg.norm(m),PA)

    return d < 15

def defensePosition(world):
    posBallX = world.ball.x
    posBallY = world.ball.y

    posRobX = world.robot.x
    posRobY = world.robot.y
    posRobTheta = world.robot.theta

    posProtectX = world.posProtectX
    posProtectY = world.posProtectY
 
    if posBallY > posRobY: #Check that the robot is at least goal side
        m = np.array([posBallX-posProtectX,posBallY-posProtectY]) #Direction vector of line
        A = np.array([posProtectX,posProtectY]) #Origin of line
        P = np.array([posRobX,posRobY]) #Robot point
        PA = P-A #From goal to rob

        pose = np.dot(PA,m)/np.linalg.norm(m)**2*m + A
        point = (pose[0],pose[1],np.arctan2(pose[0],pose[1]))
    else: point = (posProtectX,posBallY,np.pi/2) #go to goal, Can add more functionality

    return updateGoalPositions(world,point[0],point[1],posRobTheta)

def scoringPosition(world):  #Retruns a position and orientation of the robot that is in line with the ball and goal
    posTargetY = world.posTargetY
    posTargetX = world.posTargetX

    posBallY = world.ball.y
    posBallX = world.ball.x
    
    theta = np.arctan2(posTargetY - posBallY, posTargetX - posBallY)
    x = posBallX - 50*np.cos(theta)
    y = posBallY - 50*np.sin(theta)

    world.posScoringX = x
    world.posScoringY = y
    world.posScoringTheta = theta

    return world

def scoringCheck(world):
    world = scoringPosition(world)
    posRobX = world.robot.x
    posRobY = world.robot.y
    posRobTheta = world.robot.theta

    posX = world.posScoringX
    posY = world.posScoringY
    posTheta = world.posScoringTheta

    distance = 100 #Maximum distance
    angle = np.pi/16 #Maximum angle
    return abs(posRobTheta-posTheta) < angle and np.sqrt((posRobX - posX)**2 + (posRobY-posY[1])**2) < distance
