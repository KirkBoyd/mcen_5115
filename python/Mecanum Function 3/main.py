#Libraries
import numpy as np
import time
from numpy.core.numeric import ones
from gpiozero import Button
from gpiozero import LED
import gpiozero
import serial
import signal

#Helper Files
import world
import kinematics
import debugging
import communication
import navigation

# GPIO Variables
team = 'green'
button = Button(16)
buttonGRN = Button(5)
buttonBLU = Button(6)
ledBLU = LED(23)
imuCounter = 0
ledGRN = LED(24)
radCounter = 0


    
#Serial Communication Initialization and resetting
serMotors = serial.Serial('/dev/ttyACM0',9600,write_timeout=.05,timeout=.5) #IMU and Motors
serRadio = serial.Serial('/dev/ttyACM1',9600,write_timeout=.5,timeout=.5) #Radio
time.sleep(1)

serMotors.flush()
serMotors.reset_output_buffer()
serMotors.reset_input_buffer()

serRadio.flush()
serRadio.reset_output_buffer()
serRadio.reset_input_buffer()
connected = True

#Serial Communication functions
def pull(world):    
    if (serRadio.in_waiting > 0):
        try:
            #print("trying to read mmmmmmmRadio packet")
            inPacket = serRadio.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
            world.parseRadio(inPacket)
            #sprint(inPacket)
        except UnicodeDecodeError:
            #print("Invalid Packet")
            return world
    return world

def push(world): #pushes data TO the arduino from the pi (Complete)
    robot = world.robot
    speeds = ""
    directions = ""
    if serMotors.out_waiting ==0:
        for i in range(4):
            speeds = speeds + str(int(robot.speeds[i]))
            directions = directions + str(int(robot.directions[i]))
            if i < 3:
                speeds = speeds + "-"
                directions = directions + "-"
                
        packetToSend = "<MOT|" + speeds + "-" + directions + "->\n"

        try:
            serMotors.write(packetToSend.encode('utf-8'))
            #print('Sent:' + packetToSend)
        except serial.serialutil.SerialTimeoutException:
            serMotors.reset_output_buffer
            serMotors.reset_output_buffer
            print("Push Serial Timeout")
            return
    else: serMotors.reset_output_buffer
    
def playSoccer(team,opponentColor,posTargetx,posTargety,posProtectx,posProtecty):
    soccerWorld = world.worldClass(team,posTargetx,posTargety,posProtectx,posProtecty)
    soccerWorld = navigation.updateGoalPositions(soccerWorld,300,50,0)
    
    try:
        while True:
            soccerWorld = pull(soccerWorld)
            if navigation.defense(soccerWorld): #Ball is on our half
                if navigation.checkDefensive(soccerWorld): #We are in a protective position
                    if  navigation.scoringCheck(soccerWorld): #Robot is touching ball
                        navigation.updateGoalPositions(soccerWorld,soccerWorld.posTargetX,soccerWorld.posTargetY,soccerWorld.robot.theta)
                    else: #Robot is far from ball
                        soccerWorld = navigation.scoringPosition(soccerWorld)
                        soccerWorld = navigation.updateGoalPositions(soccerWorld,soccerWorld.posScoringX,soccerWorld.posScoringY,soccerWorld.posScoringTheta)
                else: #We are not in a protective position
                        navigation.defensePosition(soccerWorld)
            else: #Ball is on their half
                if  navigation.scoringCheck(soccerWorld): #Robot is touching ball
                        navigation.updateGoalPositions(soccerWorld,soccerWorld.posTargetX,soccerWorld.posTargetY,soccerWorld.robot.theta)
                else: #Robot is far from ball
                    soccerWorld = navigation.scoringPosition(soccerWorld)
                    soccerWorld = navigation.updateGoalPositions(soccerWorld,soccerWorld.posScoringX,soccerWorld.posScoringY,soccerWorld.posScoringTheta)  
            push(soccerWorld)
    except KeyboardInterrupt:
        print("turds")
        stop = "<STOP>"
        serMotors.write(stop.encode('utf-8'))
        
def readIMU():
    val = 0
    for i in range(9):
        if binaryButtons[i].is_active:
            x = 1
        else: x = 0
        val = val + x*2**(8-i)
        #print(str(i) + "............." + str(x))
    return val

def testDebug(team,opponentColor,posTargetx,posTargety,posProtectx,posProtecty):
    # NEW BINARY STUFF #
    binaryPins = [17,27,22,13,19,26,20,21,25]
    binaryButtons = [0,0,0,0,0,0,0,0,0]
    binaryVals = [0,0,0,0,0,0,0,0,0]

    for i in range(9): # initialize buttons as pins
        binaryButtons[i] = gpiozero.DigitalInputDevice(binaryPins[i],pull_up = None, active_state = True)
        
    try:
        debugWorld = world.worldClass(team,opponentColor,posTargetx,posTargety,posProtectx,posProtecty)
        while True:
            angle = 0
            for i in range(9):
                if binaryButtons[i].is_active:
                    angle = angle + 2**(8-i)
            angle = angle /180*np.pi
            if not debugWorld.robot.imuBiasRecieved:
                debugWorld.robot.imuBias = angle
                debugWorld.robot.imuBiasRecieved = True
            angle = angle - debugWorld.robot.imuBias
            if angle > np.pi:
                angle = angle - 2*np.pi
                
            debugWorld.robot.theta = angle
            
            debugWorld = pull(debugWorld)
            debugWorld = navigation.updateGoalPositions(debugWorld,debugWorld.robot.x,debugWorld.robot.y,0)
            debugWorld = kinematics.updateGoalSpeeds(debugWorld)
            debugWorld = kinematics.updateMotorSpeeds(debugWorld)
            debugging.printRobotCoords(debugWorld)
            debugging.printGoalSpeeds(debugWorld)
            push(debugWorld)
    except KeyboardInterrupt:
        print("turds")
        serMotors.write(b"<STP|>")
            
    pass
    
if __name__ == '__main__': #Main Loop
    print('------------------------')
    print('Waiting for start button')
    print('------------------------')
    while not button.value:
        if buttonGRN.is_pressed:
            team = 'green'
            opponentColor = 'blue'
            posTargetx = 192
            posTargety = 499
            posProtectx = 183 #Our goal
            posProtecty = -26 #Our Goal
            ledGRN.on()
            ledBLU.off()
        elif buttonBLU.is_pressed:
            team = 'blue'
            opponentColor = 'green'
            posTargetx = 183
            posTargety = -26
            posProtectx = 192 #Our goal
            posProtecty = 499 #Our Goal
            ledBLU.on()
            ledGRN.off()
    print("Started")
    testDebug(team,opponentColor,posTargetx,posTargety,posProtectx,posProtecty)