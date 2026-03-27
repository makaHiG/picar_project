#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

class Pin(object):
    OUT = GPIO.OUT
    IN = GPIO.IN
    IRQ_FALLING = GPIO.FALLING
    IRQ_RISING = GPIO.RISING
    IRQ_RISING_FALLING = GPIO.BOTH
    PULL_UP = GPIO.PUD_UP
    PULL_DOWN = GPIO.PUD_DOWN
    PULL_NONE = None

    _dict = {
        "BOARD_TYPE": 12,
    }

    _dict_1 = {
        "D0":  17,
        "D1":  18,
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25,
        "D7":  4,
        "D8":  5,
        "D9":  6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  19,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 21,
    }

    _dict_2 = {
        "D0":  17,
        "D1":   4, # Changed
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25, # Removed
        "D7":   4, # Removed
        "D8":   5, # Removed
        "D9":   6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  25, # Changed
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST":  5, # Changed
    }

    def __init__(self, *value):
        super().__init__()
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.check_board_type()

        if len(value) > 0:
            pin = value[0]
        if len(value) > 1:
            mode = value[1]
        else:
            mode = None
        if len(value) > 2:
            setup = value[2]
        else:
            setup = None
        if isinstance(pin, str):
            try:
                self._board_name = pin
                self._pin = self.dict()[pin]
            except Exception as e:
                print(e)
                self._error('Pin should be in %s, not %s' % (self._dict.keys(), pin))
        elif isinstance(pin, int):
            self._pin = pin
        else:
            self._error('Pin should be in %s, not %s' % (self._dict.keys(), pin))
        self._value = 0
        self.init(mode, pull=setup)
        # self._info("Pin init finished.")
        
    def check_board_type(self):
        type_pin = self.dict()["BOARD_TYPE"]
        GPIO.setup(type_pin, GPIO.IN)
        if GPIO.input(type_pin) == 0:
            self._dict = self._dict_1
        else:
            self._dict = self._dict_2

    def init(self, mode, pull=PULL_NONE):
        self._pull = pull
        self._mode = mode
        if mode != None:
            if pull != None:
                GPIO.setup(self._pin, mode, pull_up_down=pull)
            else:
                GPIO.setup(self._pin, mode)

    def dict(self, *_dict):
        if len(_dict) == 0:
            return self._dict
        else:
            if isinstance(_dict, dict):
                self._dict = _dict
            else:
                self._error(
                    'argument should be a pin dictionary like {"my pin": ezblock.Pin.cpu.GPIO17}, not %s' % _dict)

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):
        if len(value) == 0:
            self.mode(self.IN)
            result = GPIO.input(self._pin)
            # self._debug("read pin %s: %s" % (self._pin, result))
            return result
        else:
            value = value[0]
            self.mode(self.OUT)
            GPIO.output(self._pin, value)
            return value

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        if len(value) == 0:
            return self._mode
        else:
            mode = value[0]
            self._mode = mode
            GPIO.setup(self._pin, mode)

    def pull(self, *value):
        return self._pull

    def irq(self, handler=None, trigger=None, bouncetime=200):
        self.mode(self.IN)
        GPIO.add_event_detect(self._pin, trigger, callback=handler, bouncetime=bouncetime)

    def name(self):
        return "GPIO%s"%self._pin

    def names(self):
        return [self.name, self._board_name]

    class cpu(object):
        GPIO17 = 17
        GPIO18 = 18
        GPIO27 = 27
        GPIO22 = 22
        GPIO23 = 23
        GPIO24 = 24
        GPIO25 = 25
        GPIO26 = 26
        GPIO4  = 4
        GPIO5  = 5
        GPIO6  = 6
        GPIO12 = 12
        GPIO13 = 13
        GPIO19 = 19
        GPIO16 = 16
        GPIO26 = 26
        GPIO20 = 20
        GPIO21 = 21

        def __init__(self):
            pass


class Ultrasonic_Avoidance(object):
    # timeout = 0.05

    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin)
        self.echo = Pin(echo_pin)

    def distance(self):
        ##timeout=0.01  default
        timeout=0.01
        
        self.trig.low()
        time.sleep(0.001)
        self.trig.high()
        time.sleep(0.000015)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        #print("Check1 ", self.echo.value())
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        
        #print("Check2 ", self.echo.value())

        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        ##Added by me---
        # if pulse_start == 0 or pulse_end == 0: 
        #     return -3
        # if during <= 0 or during > 0.03:
        #    return -3
        ##----Added by me
        cm = round(during * 340 / 2 * 100, 2)
        #print(cm)
        return cm

    def get_distance(self, mount = 5):
        sum = 0
        for i in range(mount):
            a = self.distance()
            #print('    %s' % a)
            sum += a
        return int(sum/mount)


    def less_than(self, alarm_gate):
        dis = self.get_distance()
        status = 0
        if dis >=0 and dis <= alarm_gate:
            status = 1
        elif dis > alarm_gate:
            status = 0
        else:
            status = -1
        #print('distance =',dis)
        #print('status =',status)
        return status

def test():
    UA1 = Ultrasonic_Avoidance('D13', 'D10')  # First sensor
    UA2 = Ultrasonic_Avoidance('D14', 'D12')  # Second sensor, example pins
    threshold = 10
    while True:
        distance1 = UA1.get_distance()
        status1 = UA1.less_than(threshold)
        distance2 = UA2.get_distance()
        status2 = UA2.less_than(threshold)
        
        print('Sensor 1:')
        if distance1 != -1:
            print('distance', distance1, 'cm')
        else:
            print(False)
        if status1 == 1:
            print("Less than %d" % threshold)
        elif status1 == 0:
            print("Over %d" % threshold)
        else:
            print("Read distance error.")
        
        print('Sensor 2:')
        if distance2 != -1:
            print('distance', distance2, 'cm')
        else:
            print(False)
        if status2 == 1:
            print("Less than %d" % threshold)
        elif status2 == 0:
            print("Over %d" % threshold)
        else:
            print("Read distance error.")
        
        time.sleep(0.2)

if __name__ == '__main__':
    test()
