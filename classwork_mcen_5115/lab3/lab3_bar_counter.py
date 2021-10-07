import RPi.GPIO as GPIO
import time   
GPIO.setmode(GPIO.BCM)
GPIO.setup(19, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
numlit = 0
arr = [18, 23, 24, 25, 12, 16, 20, 21]


def switchState1(channel):
    global numlit
    global arr
    for i in arr[0:numlit+1]:
        GPIO.output(i, True)
    numlit += 1
    if numlit >= 9:
        switchState2(6)

def switchState2(channel):
    global numlit
    global arr
    for i in arr:
        GPIO.output(i, False)
    numlit = 0


GPIO.add_event_detect(19, GPIO.RISING, callback = switchState1, bouncetime = 300)    
GPIO.add_event_detect(6, GPIO.RISING, callback = switchState2, bouncetime = 300)    

try:
    while True:
        time.sleep(.001)
except KeyboardInterrupt:
    print("turds")

print("ending")
GPIO.cleanup()