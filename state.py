import math

import numpy as np


class Mode:
    IDLE="idle"
    MANUAL ="manual"
    DIRECTIONAL_MOVE="directional move"
    ORIENTING = "orienting"
    SPINNING = "spinning"

class RobotState:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.Sensors = SensorState()
        self.direction = 0
        self.rotation = 0
        self.mode = Mode.IDLE
        self.targetAngle = 0
        self.corridorAngle=0
        self.center_errors =[]
        self.align_errors = []
        self.readings = []
        self.right_distance = 100
        self.left_distance = 100
        self.front_distance = 100
        self.scan = ScanState()
        self.spinn = SpinnState()
        self.lastPhotoSpot = (0,0)
        self.realRun = False
class ScanState:
    def __init__(self):
        self.readings=[]
        self.active = False
        self.startRotation = 0

class SensorState:
    def __init__(self):
        self.right_points = []
        self.left_points = []
        self.front_points = []
        self.sideways_offset = 20
    def add_reading(self, sensor, distance, x, y, rotation):
        angle_rad = math.radians(rotation)
        if sensor == "front":
            fx = x + (distance+20) * math.cos(angle_rad)
            fy = y + (distance+20) * math.sin(angle_rad)
            self.front_points.append((fx, fy))
            if(len(self.front_points)>100):
                self.front_points.pop(0)
        elif sensor == "left":
            lx = x + (distance+self.sideways_offset)* math.cos(angle_rad + math.pi/2)                
            ly = y + (distance+self.sideways_offset) * math.sin(angle_rad + math.pi/2)
            self.left_points.append((lx, ly))
            if(len(self.left_points)>100):
                self.left_points.pop(0)
        elif sensor == "right":
            rx = x + (distance+self.sideways_offset)  * math.cos(angle_rad - math.pi/2)
            ry = y + (distance+self.sideways_offset) * math.sin(angle_rad - math.pi/2)
            self.right_points.append((rx, ry))
            if(len(self.right_points)>100):
                self.right_points.pop(0)
    def get_leftWallAngle(self):
        if len(self.left_points)<2:
            return None
        else:
            return self.fit_line_and_error(self.left_points)
    def get_rightWallAngle(self):
        if len(self.right_points)<2:
            return None
        else:
            return self.fit_line_and_error(self.right_points)
    def fit_line_and_error(self, points):
        if len(points) < 2:
            return None, None

        pts = np.array(points[-10:])  # last 10 points

        # --- compute mean ---
        mean = np.mean(pts, axis=0)

        # --- center data ---
        centered = pts - mean

        # --- covariance ---
        cov = np.cov(centered.T)

        # --- eigen decomposition ---
        eigenvalues, eigenvectors = np.linalg.eig(cov)

        # largest eigenvector = line direction
        idx = np.argmax(eigenvalues)
        direction = eigenvectors[:, idx]

        # --- angle of line ---
        angle = math.atan2(direction[1], direction[0])

        # --- compute perpendicular distances (error) ---
        # normal vector to the line
        normal = np.array([-direction[1], direction[0]])

        distances = np.abs(centered @ normal)

        # RMSE (noise metric)
        rmse = np.sqrt(np.mean(distances**2))

        return angle, rmse
class SpinnState:
    def __init__(self):
        self.batchfolder = "batch"
        self.panoramafolder = "panorama"
        self.panoramacounter = 0
        self.targetRotation = 0
        self.active = False
        self.stepCount = 0
        self.maxSteps = 12
        self.startRotation = 0

        
