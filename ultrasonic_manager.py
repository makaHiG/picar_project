import threading
import time
from .ultrasonic_module import Ultrasonic_Avoidance
from .ultrasonic_avoidance_3pin import Ultrasonic_Avoidance2

class UltrasonicManager:
    def __init__(self,front,left,right):
        self.front = front
        self.left = left
        self.right = right
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        #self.thread.start()
        self.left_distance = 0
        self.right_distance = 0
        self.front_distance = 0

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def run(self):
        while True:
            self.front_distance = self.front.get_distance()
            time.sleep(0.06)
            self.left_distance = self.left.get_distance()
            time.sleep(0.06)
            self.right_distance = self.right.get_distance()
            time.sleep(0.06)

            