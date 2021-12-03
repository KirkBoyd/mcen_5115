import numpy as np
import time
from numpy.core.numeric import ones
from gpiozero import Button
from gpiozero import LED
import serial
import signal

def playSoccer():
    pass

def testdebug():
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
    pass