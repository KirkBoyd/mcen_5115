import numpy as np
import tkinter as tk
import serial
import world
import usb.core
import usb.util
import time
import os
import robotFile
os.system('cls' if os.name == 'nt' else 'clear')

try:
    serPort0 = serial.Serial('/dev/ttyACM0',9600,write_timeout=.05,timeout=.5)
except:
    print("Error Opening Port")

def push(command,data):
    if serPort0.out_waiting ==0:
        packet = "<" + command + "-"
        for element in data:
            packet = packet + str(element) + "-"
        packet = packet + ">\n"

        try:
            serPort0.write(packet.encode('utf-8'))
            #print('Sent:' + packet)
        except serial.serialutil.SerialTimeoutException:
            serPort0.reset_output_buffer
            serPort0.reset_output_buffer
            print("Push Serial Timeout")
            return
    else: serPort0.reset_output_buffer


def keypress(event):
    x = event.char
    if x == "q":
        robot.calculateMotorSpeeds(5000,5000,0)
    elif x == "w":
        robot.calculateMotorSpeeds(0,5000,0)
    elif x == "e":
        robot.calculateMotorSpeeds(5000,-5000,0)
    elif x == "a":
        robot.calculateMotorSpeeds(5000,0,0)
    elif x == "s":
        robot.calculateMotorSpeeds(0,0,0)
    elif x == "d":
        robot.calculateMotorSpeeds(-5000,0,0)
    elif x == "z":
        robot.calculateMotorSpeeds(5000,-5000,0)
    elif x == "x":
        robot.calculateMotorSpeeds(0,-5000,0)
    elif x == "c":
        robot.calculateMotorSpeeds(-5000,-5000,0)
    elif x == "k":
        robot.calculateMotorSpeeds(0,0,100)
    elif x == "l":
        robot.calculateMotorSpeeds(0,0,-100)
    speeds = robot.wheelsSpeeds
    directions = np.array([0,0,0,0])
    i = 0
    for speed in speeds:
        if speed > 0:
            directions[i] = 1
        else:
            directions[i] = 0
        speeds[i] = abs(speed)
        i += 1
    
    toPush = np.append(speeds,directions)
    robot.printWheelSpeeds()
    push("MOT",toPush)

#bind all major keys and call our keypress handler
window = tk.Tk()
keys = ["q","w","e","a","s","d","z","x","c","k","l"]
for key in keys:
    window.bind(key, keypress)

#start the loop
robot = robotFile.robotClass()

window.mainloop()