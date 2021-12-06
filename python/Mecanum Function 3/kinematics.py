import numpy as np
    
def updateGoalSpeeds(world): #(Inprogress)
    currentX = world.robot.x
    currentY = world.robot.y
    currentTheta = world.robot.theta
    
    worldGoalX = world.robot.goalX
    worldGoalY = world.robot.goalY
    goalTheta = world.robot.goalTheta
    
    robotCoords = world.world2robot(worldGoalX,worldGoalY)
    #print(robotCoords)
    goalX = robotCoords[0]
    goalY = robotCoords[1]
    
    deltaTheta = goalTheta-currentTheta
    
    goalVelocityX = goalX
    goalVelocityY = goalY
    goalVelocityTheta = deltaTheta
    
    world.robot.goalVelocityX = goalVelocityX
    world.robot.goalVelocityY = -goalVelocityY
    world.robot.goalVelocityTheta =goalVelocityTheta
    return world

def updateMotorSpeeds(world):
    vx = world.robot.goalVelocityX
    vy = world.robot.goalVelocityY
    vTrans = (vx**2 + vx**2)**.5
    wz = -world.robot.goalVelocityTheta #Omega Z
    if vTrans < 25 and abs(wz) < .2:
        speedRatio = .15
    elif vTrans < 100:
        speedRatio = .3
    else:
        speedRatio = .75
    
    r = world.robot.r
    lx = world.robot.lx
    ly = world.robot.ly
    
    wz = wz*0.5

    omega0 = (vx-vy-(lx+ly)*wz)/r
    omega1 = (vx+vy+(lx+ly)*wz)/r
    omega2 = (vx+vy-(lx+ly)*wz)/r
    omega3 = (vx-vy+(lx+ly)*wz)/r
    
    omegas = np.array([omega0,omega1,omega2,omega3])
    directions = np.array(omegas > 0)
    
    absoluteOmegas = abs(omegas)
    maxOmega = max(omegas)
    
    if maxOmega > 0:
        normalizedOmegas = np.interp(absoluteOmegas,[0, maxOmega],[75,254])
    else:
        normalizedOmegas = absoluteOmegas*0
    
    world.robot.speeds = normalizedOmegas*speedRatio
    world.robot.directions = directions
    return world