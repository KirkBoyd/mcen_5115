def printRadio(world):
    print("Parse radio data: " + str(world.radioData))
    pass

def printRobotCoords(world):
    print("Robot X: " + str(world.robot.x) + " Robot Y: " + str(world.robot.y) + " Robot Theta: " + str(world.robot.theta))
    pass

def printGoalSpeeds(world):
    print("Goal VX: " + str(world.robot.goalVelocityX) + " Goal VY: " + str(world.robot.goalVelocityY) + " Goal VTheta: " + str(world.robot.goalVelocityTheta))
    pass

def printMotorSpeeds(world):
    print("Robot Motor Speeds: " + str(world.robot.speeds) + " Robot Motor Directions: " + str(world.robot.directions))
    pass