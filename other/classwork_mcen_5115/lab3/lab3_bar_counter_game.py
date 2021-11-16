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
i = 0
won = False


def switchState1(pin):
    global numlit
    global arr
    GPIO.output(arr[pin], True)

def switchState2(channel):
    global numlit
    global arr
    for i in arr:
        GPIO.output(i, False)
    numlit = 0

def switchState3(channel):
    global i 
    global won 
    print(i)
    if (i == 3):
        switchState1(3)
        won = True
        time.sleep(10)
    




GPIO.add_event_detect(19, GPIO.RISING, callback = switchState3, bouncetime = 300)    
# GPIO.add_event_detect(6, GPIO.RISING, callback = switchState2, bouncetime = 300)    

try:
    while not won:
        # if not GPIO.input(19):
        #     if (i == 3):
        #         switchState1(3)
        #         won = True
        #         time.sleep(6)
        time.sleep(0.1)
        switchState2(6)
        switchState1(i)
        i += 1
        if(i >= 8):
            i = 0
        if won:
            break

except KeyboardInterrupt:
    print("turds")

print("ending")
GPIO.cleanup()