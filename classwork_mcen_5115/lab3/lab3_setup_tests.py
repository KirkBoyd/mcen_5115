import RPi.GPIO as GPIO
import time   
GPIO.setmode(GPIO.BCM)
GPIO.setup(19, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

def butt(channel):
    GPIO.output(18, True)
    time.sleep(0.001)
    GPIO.output(18, False)


GPIO.add_event_detect(19, GPIO.RISING, callback = butt, bouncetime = 300)    
try:
    while True:
        time.sleep(.001)
except KeyboardInterrupt:
    print("turds")

print("ending")
GPIO.cleanup()