#Libraries
import numpy as np
import time
from numpy.core.numeric import ones
from gpiozero import Button
from gpiozero import LED
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
ledIMU = LED(25)
ledRAD = LED(13)

#Serial Communication Initialization and resetting
serMotors = serial.Serial('/dev/ttyACM2',9600,write_timeout=.05,timeout=.5) #IMU and Motors
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
    if (serMotors.in_waiting > 0): 
        try:
            #print("trying to read IMU packet")
            inPacket = serMotors.readline().decode("utf-8").replace("\n", "") #Read in line, convert to string, remove new line character
            #print(inPacket)
            world.parseImu(inPacket)
            #print("The f word")
        except UnicodeDecodeError:
            print("Invalid Packet")
            return world
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
    
def playSoccer(team,posTargetx,posTargety,posProtectx,posProtecty):
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

def testDebug(team,posTargetx,posTargety,posProtectx,posProtecty):
    try:
        debugWorld = world.worldClass(team,posTargetx,posTargety,posProtectx,posProtecty)
        debugWorld = navigation.updateGoalPositions(debugWorld,300,50,0)
        i = 1
        while True:
            debugWorld = pull(debugWorld)
            debugWorld = kinematics.updateGoalSpeeds(debugWorld)
            debugWorld = kinematics.updateMotorSpeeds(debugWorld)
            debugging.printRobotCoords(debugWorld)
            push(debugWorld)
            #print("loop", i)
            #i += 1
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
            posTargetx = 192
            posTargety = 499
            posProtectx = 183 #Our goal
            posProtecty = -26 #Our Goal
            ledGRN.on()
            ledBLU.off()
        elif buttonBLU.is_pressed:
            team = 'blue'
            posTargetx = 183
            posTargety = -26
            posProtectx = 192 #Our goal
            posProtecty = 499 #Our Goal
            ledBLU.on()
            ledGRN.off()
    print("Started")
    testDebug(team,posTargetx,posTargety,posProtectx,posProtecty)