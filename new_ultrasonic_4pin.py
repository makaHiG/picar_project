# import libraries
import RPi.GPIO as GPIO
import time
 


class Ultrasonic_4pin():
    def __init__(self,trig, echo):
        self.ECHO = echo
        self.TRIG = trig
        # GPIO Modus (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)
        
        # Set direction of GPIO pins (IN --> Input / OUT --> Output)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def distance(self):
        # set trigger to HIGH
        GPIO.output(self.TRIG, True)
    
        # set trigger after 0.01 ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)
    
        startTime = time.perf_counter()
        arrivalTime = time.perf_counter()
        timeout = 0.03  # 30 ms timeout for no response
        # store startTime
        while GPIO.input(self.ECHO) == 0:
            startTime = time.perf_counter()
    
        # store arrivalTime
        while GPIO.input(self.ECHO) == 1:
            arrivalTime = time.perf_counter()
            if arrivalTime - startTime > timeout:
                return -2  # Timeout occurred, no object detected
    
        # Time difference between start and arrival
        timeElapsed = arrivalTime - startTime
        # multiply by the speed of sound (34300 cm/s)
        # and divide by 2, there and back again
        distance = (timeElapsed * 34300) / 2
    
        return round(distance,2)
 
# assign GPIO Pins
GPIO_TRIGGER = 16
GPIO_ECHO = 12
 
 
if __name__ == '__main__':
    ultrasonic = Ultrasonic_4pin(GPIO_TRIGGER, GPIO_ECHO)
    try:
        while True:
            distance = ultrasonic.distance()
            print ("Measured distance = %.1f cm" % distance)
            time.sleep(0.06)
 
        # When canceling with CTRL+C, resetting
    except KeyboardInterrupt:
        print("Measurement stopped by user")
        GPIO.cleanup()