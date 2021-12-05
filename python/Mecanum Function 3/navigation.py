def updateGoalPositions(world,goalX,goalY,goalTheta):
    world.robot.goalX = goalX
    world.robot.goalY = goalY
    world.robot.goalTheta = goalTheta
    return world