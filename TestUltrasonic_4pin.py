# import libraries
import RPi.GPIO as GPIO
import time
 
# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
# assign GPIO Pins
GPIO_TRIGGER = 16
GPIO_ECHO = 12
 
# Set direction of GPIO pins (IN --> Input / OUT --> Output)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
 
def get_distance():
    # set trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set trigger after 0.01 ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    startTime = time.time()
    arrivalTime = time.time()
 
    # store startTime
    while GPIO.input(GPIO_ECHO) == 0:
        startTime = time.time()
 
    # store arrivalTime
    while GPIO.input(GPIO_ECHO) == 1:
        arrivalTime = time.time()
 
    # Time difference between start and arrival
    timeElapsed = arrivalTime - startTime
    # multiply by the speed of sound (34300 cm/s)
    # and divide by 2, there and back again
    distance = (timeElapsed * 34300) / 2
 
    return distance
 
if __name__ == '__main__':
    try:
        while True:
            distance = get_distance()
            print ("Measured distance = %.1f cm" % distance)
            time.sleep(1)
 
        # When canceling with CTRL+C, resetting
    except KeyboardInterrupt:
        print("Measurement stopped by user")
        GPIO.cleanup()