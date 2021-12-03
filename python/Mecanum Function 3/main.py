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
serMotors = serial.Serial('/dev/ttyACM0',9600,write_timeout=.5,timeout=.5) #IMU and Motors
serRadio = serial.Serial('/dev/ttyACM1',9600,write_timeout=.5,timeout=.5) #Radio
time.sleep(1)

serMotors.flush()
serMotors.reset_output_buffer()
serMotors.reset_input_buffer()

serRadio.flush()
serRadio.reset_output_buffer()
serRadio.reset_input_buffer()
connected = True


def playSoccer():
    pass

def testDebug():
    try:
        debugWorld = world.worldClass()
        while True:
            debugWorld = communication.pull(debugWorld,serMotors,serRadio)
            #debugging.printRadio(debugWorld)
            print("loop")
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
    testDebug()