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
    world.robot.goalVelocityTheta = goalVelocityTheta
    return world

def updateMotorSpeeds(world):
    vx = world.robot.goalVelocityX
    vy = world.robot.goalVelocityY
    vTrans = (vx**2 + vx**2)**.5
    wz = -world.robot.goalVelocityTheta #Omega Z
    if vTrans < 25:
        speedRatio = .5
    elif vTrans == 0:
        speedRatio = .5
    elif vTrans < 50:
        speedRatio = .75
    else:
        speedRatio = 1
    
    r = world.robot.r
    lx = world.robot.lx
    ly = world.robot.ly
    #print(wz)
    wz = wz*0.01 * abs(vTrans+1)

    omega0 = (vx-vy-(lx+ly)*wz)/r
    omega1 = (vx+vy+(lx+ly)*wz)/r
    omega2 = (vx+vy-(lx+ly)*wz)/r
    omega3 = (vx-vy+(lx+ly)*wz)/r
    #print(wz)
    omegas = np.array([omega0,omega1,omega2,omega3])
    #print(omegas)
    directions = np.array(omegas > 0)
    
    absoluteOmegas = abs(omegas)
    maxOmega = max(omegas)
    
    if maxOmega > 0:
        normalizedOmegas = np.interp(absoluteOmegas,[0, maxOmega],[0,254])
    else:
        normalizedOmegas = np.interp(absoluteOmegas,[0, maxOmega],[0,254])
    
    world.robot.speeds = normalizedOmegas*speedRatio
    world.robot.directions = directions
    return world