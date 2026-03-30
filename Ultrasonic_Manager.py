import threading
import time
from picar import ultrasonic_module
from picar import Ultrasonic_Avoidance2

class Ultrasonic_Manager:
    def __init__(self,front,left,right):
        self.front = front
        self.left = left
        self.right = right
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def run(self):
        while True:
            self.front_distance = ultrasonic_module.get_distance(self.front)
            time.sleep(0.06)
            self.left_distance = Ultrasonic_Avoidance2.get_distance(self.left)
            time.sleep(0.06)
            self.right_distance = Ultrasonic_Avoidance2.get_distance(self.right)
            time.sleep(0.06)

            