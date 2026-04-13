import multiprocessing as mp
import time
from .ultrasonic_avoidance_3pin import Ultrasonic_Avoidance2 as UA2
from .new_ultrasonic_4pin import Ultrasonic_4pin as UA4
class UltrasonicManager:
    def __init__(self, front_pin, left_pins, right_pins, que):
        self.que = que
        self.front_pin = front_pin
        self.left_pins = left_pins
        self.right_pins = right_pins
        self.running = mp.Value('b', False)
        self.process = mp.Process(target=self.run)
        
        self.left_distance = 0
        self.right_distance = 0
        self.front_distance = 0
        self.right_trend = 0
        self.left_trend = 0
        self.right_values = []
        self.left_values = []
        self.right_confidence=1
        self.left_confidence=1
    def start(self):
        self.running.value = True
        self.process.start()

    def stop(self):
        self.running.value = False
        self.process.join()

    # def get_distance_left():
    #     return()

    def HandleUltrasonicData(self,dist, lst):
        
        if 0<dist<1000:
            lst.append(dist)
            
        elif dist == -2:
            lst.append(-2)
        else:
            
            print(f"Faulty reading {dist } from {'Right'if lst is self.right_values else 'Left'}")
            
            lst.append(-3)  # Append a default value for faulty readings
 
        
        if(len(lst)>5):
                lst.pop(0)
        # if manual_drive.debug["sensors"]:
        #         print(self.left_distance,"|",self.front_distance,"|",self.right_distance)



    def run(self):
        self.front = UA2(self.front_pin)
        self.left = UA4(self.left_pins[0], self.left_pins[1])
        self.right = UA4(self.right_pins[0], self.right_pins[1])
        
        self.left_values = []
        self.right_values = []
        while self.running.value:
            a = time.time()
            
            self.front_distance = self.front.get_distance()
            time.sleep(0.01)
            self.HandleUltrasonicData(self.left.distance(),self.left_values)
            self.left_distance = (sorted(self.left_values)[len(self.left_values)//2]) if self.left_values else 0
            time.sleep(0.01)
            self.HandleUltrasonicData( self.right.distance(),self.right_values)
            self.right_distance = (sorted(self.right_values)[len(self.right_values)//2]) if self.right_values else 0
            #print(self.right_distance," | ",self.left_distance )
            self.que.put((self.left_distance,self.front_distance,self.right_distance))
            time.sleep(0.01)

            