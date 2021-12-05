import numpy as np
    
def updateGoalSpeeds(world): #(Inprogress)
    currentX = world.robot.x
    currentY = world.robot.y
    currentTheta = world.robot.theta
    
    worldGoalX = world.robot.goalX
    worldGoalY = world.robot.goalY
    goalTheta = world.robot.goalTheta
    
    robotCoords = world.world2robot(worldGoalX,worldGoalY)
    goalX = robotCoords[0]
    goalY = robotCoords[1]
    
    deltaX = goalX - currentX
    deltaY = goalY - currentY
    deltaTheta = goalTheta-currentTheta
    
    goalVelocityX = deltaX
    goalVelocityY = deltaY
    goalVelocityTheta = deltaTheta
    
    world.robot.goalVelocityX = goalVelocityX
    world.robot.goalVelocityY = goalVelocityY
    world.robot.goalVelocityTheta =goalVelocityTheta
    return world

def updateMotorSpeeds(world):
    vx = world.robot.goalVelocityX
    vy = world.robot.goalVelocityY
    wz = -world.robot.goalVelocityTheta #Omega Z
    
    r = world.robot.r
    lx = world.robot.lx
    ly = world.robot.ly
    
    omega0 = (vx-vy-(lx+ly)*wz)/r
    omega1 = (vx+vy+(lx+ly)*wz)/r
    omega2 = (vx+vy-(lx+ly)*wz)/r
    omega3 = (vx-vy+(lx+ly)*wz)/r
    
    omegas = np.array([omega0,omega1,omega2,omega3])
    directions = np.array(omegas > 0)
    
    absoluteOmegas = abs(omegas)
    maxOmega = max(omegas)
    
    if maxOmega > 0:
        normalizedOmegas = np.interp(absoluteOmegas,[-maxOmega, maxOmega],[-254,254])
    else:
        normalizedOmegas = absoluteOmegas*0
    
    world.robot.speeds = normalizedOmegas
    world.robot.directions = directions
    return world