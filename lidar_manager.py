import math
import sys
import numpy as np
import time
from multiprocessing import Process, Value
from rplidar import RPLidar
from queue import Full
from serial.tools import list_ports


PORT_NAME = "/dev/ttyUSB0"



# Qt app
robotRadius = 24
lidarOffset = np.array([10,0])
class LidarManager:
    def __init__(self, que):
        self.connected = False
        self.lidar = None
        try:
            self.lidar = RPLidar(PORT_NAME)
            self.connected = True
        except Exception as e:
            print("LiDAR unavailable:", e)

        self.que = que
        self.running = Value('b', False)
        self.process = Process(target=self.run)
        
    def start(self):
        self.running.value = True
        self.process.start()

    def stop(self):
        self.running.value = False
        self.process.join()
    


    def run(self):
        try:
            scan_iterator = self.lidar.iter_scans()        
            
            
            while self.running.value:
                scan = next(scan_iterator)
                spots = []

                for quality, angle, distance in scan:

                    # Ignore invalid readings
                    if distance <= 0:
                        continue

                    rad = math.radians(angle)

                    x = distance/10 * math.cos(rad) #mm to cm
                    y = distance/10 * math.sin(rad)

                    spots.append({
                        'pos': np.array([x, y])+lidarOffset,
                        "time": time.time()
                    })
                while not self.que.empty():
                    self.que.get_nowait()

                self.que.put(spots)
        finally:
            lidar.stop()
            lidar.disconnect()
    



    