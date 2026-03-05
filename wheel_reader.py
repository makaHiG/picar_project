import RPi.GPIO as GPIO
import time

SENSOR_PIN = 16   # change if your sensor uses another pin
pulses = 0

def magnet_detected(channel):
    global pulses
    pulses += 1

GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect(SENSOR_PIN, GPIO.FALLING, callback=magnet_detected)

try:
    while True:
        rotations = pulses / 6
        print("Pulses:", pulses, " Rotations:", rotations)
        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()
