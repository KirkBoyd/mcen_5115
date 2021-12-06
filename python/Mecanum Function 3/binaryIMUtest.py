#Libraries
import numpy as np
import time
from numpy.core.numeric import ones
from gpiozero import Button
from gpiozero import LED
import gpiozero
import serial
import signal

# NEW BINARY STUFF #
binaryPins = [17,27,22,13,19,26,20,21,25]
binaryButtons = [0,0,0,0,0,0,0,0,0]
binaryVals = [0,0,0,0,0,0,0,0,0]

for i in range(9): # initialize buttons as pins
    binaryButtons[i] = gpiozero.DigitalInputDevice(binaryPins[i],pull_up = None, active_state = True)
    

def readIMU():
    val = 0
    for i in range(9):
        if binaryButtons[i].is_active:
            x = 1
        else: x = 0
        val = val + x*2**(8-i)
        #print(str(i) + "............." + str(x))
    print(val)
    
def readBinary(binButton):
    if binButton.is_active:
        return 1
    else:
        return 0

def printBinary():
    binaryValStr = ""
    for i in range(9):
        binaryValStr = binaryValStr + str(readBinary(binaryButtons[i]))
        #print("Input " + str(i) + " is: " + str(binaryPins[i]) + " which is " + str(readBinary(binaryButtons[i])))
    
    binVal = (int(binaryValStr, 2))
    print("binary vals: " + binaryValStr)
    print("Final val is: " + str(binVal))
    
if __name__ == '__main__': #Main Loop
    while True:
        readIMU()