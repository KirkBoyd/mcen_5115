#!/usr/bin/python

import smbus
import time

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x54 


while True:
    word = bus.read_word_data(DEVICE_ADDRESS, 1)
    print(hex(word))
    if word == 0xaa55:
        print("1st loop achieved")
        word = bus.read_word_data(DEVICE_ADDRESS, 1)
        print("word: ", hex(word))
        if word == 0xaa55:
            
            print("2nd loop achieved")
            checksum = bus.read_word_data(DEVICE_ADDRESS, 1)
            signature = bus.read_word_data(DEVICE_ADDRESS, 1)
            xctr = bus.read_word_data(DEVICE_ADDRESS, 1)
            yctr = bus.read_word_data(DEVICE_ADDRESS, 1)
            width = bus.read_word_data(DEVICE_ADDRESS, 1)
            height = bus.read_word_data(DEVICE_ADDRESS, 1)
            print("signature: ", int(signature))
            print("x ctr: ", int(xctr))
            print("y ctr: ", yctr)
            print("width: ", width)
            print("height: ", height)
            
    time.sleep(0.05)