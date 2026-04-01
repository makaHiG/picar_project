import threading
import time
from queue import Queue
from .ultrasonic_module import Ultrasonic_Avoidance
from .ultrasonic_avoidance_3pin import Ultrasonic_Avoidance2
#from . import manual_drive
class UltrasonicManager:
    def __init__(self,front,left,right,que):
        self.que=que
        self.front = front
        self.left = left
        self.right = right
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        
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
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    # def get_distance_left():
    #     return()

    def HandleUltrasonicData(self,dist, lst):
        
        if 0<dist<1000:
            lst.append(dist)
            
        elif dist == -2:
            lst.append(200)
        else:
            
            print(f"Faulty reading {dist } from {'Right'if lst is self.left_values else 'Left'}")
            
            lst.append(-3)  # Append a default value for faulty readings
 
        
        if(len(lst)>5):
                lst.pop(0)
        # if manual_drive.debug["sensors"]:
        #         print(self.left_distance,"|",self.front_distance,"|",self.right_distance)



    def run(self):
        while True:
            self.front_distance = self.front.get_distance()
            time.sleep(0.06)
            self.HandleUltrasonicData(self.left.get_distance(),self.left_values)
            self.left_distance = (sorted(self.left_values)[len(self.left_values)//2]) if self.left_values else 0
            time.sleep(0.06)
            print("CheckR ", self.right.echo.value())
            print("CheckL ", self.left.echo.value() )
            #self.HandleUltrasonicData( self.right.get_distance(),self.right_values)
            #self.right_distance = (sorted(self.right_values)[len(self.right_values)//2]) if self.right_values else 0
            #self.que.put((self.left_distance,self.front_distance,self.right_distance))
            time.sleep(0.06)

            