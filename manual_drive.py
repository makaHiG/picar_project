from operator import pos
import select
import sys
import tty
import termios
import time
import math
import socket
import json
from multiprocessing import Queue
from dataclasses import dataclass
import random
from turtle import pos
from datetime import datetime
from rplidar import RPLidar
# PORT = '/dev/ttyUSB0'

# try:
#     lidar = RPLidar(PORT)
#     info = lidar.get_info()

#     print("LiDAR connected:", info)

#     lidar_connected = True

# except Exception as e:
#     print("LiDAR not available:", e)

import numpy as np
# from . import ultrasonic_manager
# from ultrasonic_manager import UltrasonicManager
# import ultrasonic_module as UA4
from .ultrasonic_manager import UltrasonicManager
from .lidar_manager import LidarManager
#from . import ultrasonic_module as UA4
from .state import RobotState,Mode,ScanState,SpinnState
from .Line_Follower import Line_Follower

import subprocess
import os
import smbus #for gyro

from picar import front_wheels, back_wheels  # PiCar-S library
#from ultrasonic_avoidance_3pin import Ultrasonic_Avoidance2 as UA2

import picar

picar.setup() ## Car will not move before this is run 
# Initialize wheels
camera_servo = front_wheels.Front_Wheels(db='config')
wheels = back_wheels.Back_Wheels(db='config')
wheels.stop()
camera_servo.turn_straight()
wheels.speed = 0
camera_servo.ready()
wheels.ready()
state = RobotState()
print(picar.back_wheels.__file__)

base_folder = os.path.expanduser("~/photos")
os.makedirs(base_folder, exist_ok=True)

lidar_queue = Queue()
lidar_manager = LidarManager(lidar_queue) 
lidar_connected = lidar_manager.connected
if(lidar_connected):
    lidar_manager.start()
# Initialize ultrasonic sensors
sensor_queue = Queue()
US_Manager = UltrasonicManager(20, (16,12), (26,19), sensor_queue)
line_follower = Line_Follower()

## SocketSetup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
IP = "255.255.255.255"
PORT = 5005
# Gyro setup
bus = smbus.SMBus(1)
MPU6050_ADDR = 0x68
direction = False
PWR_MGMT_1 = 0x6B
GYRO_ZOUT_H = 0x47
print(sys.argv)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
# Gyro functions
def read_word(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value
def read_line_follower():
    try:
        return line_follower.get_average(5)
    except IOError:
        print("Line follower read error. Please check the wiring.")
        return [0, 0, 0, 0, 0]
def read_gyro_z():
    return read_word(GYRO_ZOUT_H)
# --- Calibration ---
samples = []
for _ in range(100):
    samples.append(read_gyro_z())
    time.sleep(0.005)

offset = sum(samples) / len(samples)
#print("Offset:", offset)

# --- Integration ---
debug = {
    "wheels": False,
    "camera": False,
    "sensors": False,
    "gryo": False,
    "navigation": False
}
showOffMode = False
#print("Tracking rotation...")

# Steering & speed parameters
STEER_ANGLE = 30  # degrees left/right
SPEED = 100       # speed 0-100 default 50
TURN_TIME = 1.6
#wheels.speed = SPEED
TURN_SPEED = 50#default 30
Travel_Speed = 48*3.14/5.55 #Speed from test,cm/s



#Take Photos, angles defined in state    
def PhotoCollumn(state:RobotState=state):
    for i in range(len(state.rowAngles)):
        angle = state.rowAngles[i]
        camera_servo.smooth_turn(angle,showOff if showOffMode else None)
        state.spinn.row=i
        if(state.realRun):
            time.sleep(1) #Probably needed   #May not be needed, as warmup runs for half a second anyway
            TakePhoto(state)
        else:
            time.sleep(0.3)
    if(state.spinn.stepCount == 0 or state.spinn.stepCount == 3):
        state.spinn.row=5
        if(showOffMode):
            camera_servo.smooth_turn(180,showOff)
        else:
            camera_servo.smooth_turn(180)
        time.sleep(1)
        if(state.realRun):
            TakePhoto(state)
    camera_servo.smooth_turn(90,showOff if showOffMode else None)

def CapturePanorama(state:RobotState):
    spinn = state.spinn
    spinn: SpinnState
    if(spinn.active == False):
        if(state.realRun == True):
            # spinn.panoramafolder = os.path.join(spinn.batchfolder, "panorama"+str(spinn.panoramacounter))
            # # os.path.expanduser("~/photos")
            # os.makedirs(spinn.panoramafolder, exist_ok=True)
            base = os.path.join(spinn.batchfolder, "panorama")

            counter = 1
            folder = base

            while os.path.exists(folder):
                folder = f"{base}({counter})"
                counter += 1

            spinn.panoramafolder = folder
            os.makedirs(spinn.panoramafolder)
        spinn.stepCount = 0
        spinn.startRotation = state.rotation
        spinn.active = True
        spinn.targetRotation = spinn.startRotation
        #UpDownTest(state)
    error = spinn.targetRotation-state.rotation

    if abs(error)<0.5:
        wheels.stop()
        time.sleep(1)
        if(spinn.stepCount<spinn.maxSteps):
            
            PhotoCollumn(state)
            spinn.stepCount += 1
            spinn.targetRotation = spinn.startRotation + 360/spinn.maxSteps * spinn.stepCount
        else: 
            spinn.active=False
            spotInfo = {
                "name": f"info_panorama_{state.spinn.panoramacounter}",
                "coordinates": state.position.tolist(),
                "rotation": state.spinn.startRotation
            }
            filename = os.path.join(state.spinn.panoramafolder, f"info_panorama_{state.spinn.panoramacounter}.json")
            with open(filename, "w") as f:
                json.dump(spotInfo, f)
            state.spinn.panoramacounter+=1
            #Adding 360 since we spun a circle
            state.corridorAngle = state.corridorAngle + 360 ## Questionable
            return SteerCenter
    else:
        mod = error /30
        wheels.speed = int(min(50,max(25,TURN_SPEED*mod)))
        if error<0 :
            wheels.spinn_right()
            state.direction = 0
        else:
            wheels.spinn_left()
            state.direction = 0
    return CapturePanorama


def CapturePanoramaShowOff(state:RobotState):
    spinn = state.spinn
    spinn: SpinnState
    if(spinn.active == False):
        # if(state.realRun == True):
        #     # spinn.panoramafolder = os.path.join(spinn.batchfolder, "panorama"+str(spinn.panoramacounter))
        #     # # os.path.expanduser("~/photos")
        #     # os.makedirs(spinn.panoramafolder, exist_ok=True)
        #     base = os.path.join(spinn.batchfolder, "panorama")

        #     counter = 1
        #     folder = base

        #     while os.path.exists(folder):
        #         folder = f"{base}({counter})"
        #         counter += 1

        #     spinn.panoramafolder = folder
        #     os.makedirs(spinn.panoramafolder)
        spinn.stepCount = 0
        spinn.startRotation = state.rotation
        spinn.active = True
        spinn.targetRotation = spinn.startRotation
        #UpDownTest(state)
    error = spinn.targetRotation-state.rotation

    if abs(error)<0.5:
        wheels.stop()
        time.sleep(1)
        if(spinn.stepCount<spinn.maxSteps):
            
            PhotoCollumn(state)
            spinn.stepCount += 1
            spinn.targetRotation = spinn.startRotation + 360/spinn.maxSteps * spinn.stepCount
        else: 
            spinn.active=False
            if(state.realRun==True):
                spotInfo = {
                    "name": f"info_panorama_{state.spinn.panoramacounter}",
                    "coordinates": state.position.tolist(),
                    "rotation": state.spinn.startRotation
                }
                filename = os.path.join(state.spinn.panoramafolder, f"info_panorama_{state.spinn.panoramacounter}.json")
                with open(filename, "w") as f:
                    json.dump(spotInfo, f)
            state.spinn.panoramacounter+=1
            #Adding 360 since we spun a circle
            state.corridorAngle = state.corridorAngle + 360 ## Questionable
            return CapturePanoramaShowOff
    else:
        mod = error /30
        wheels.speed = int(min(50,max(25,TURN_SPEED*mod)))
        if error<0 :
            wheels.spinn_right()
            state.direction = 0
        else:
            wheels.spinn_left()
            state.direction = 0
    return CapturePanoramaShowOff

def MoveTo(state: RobotState, point: np.ndarray):

    px, py = point[0], point[1]

    dx = px - state.x
    dy = py - state.y

    target_angle = math.degrees(math.atan2(dy, dx))

    align_error = (target_angle - state.rotation + 180) % 360 - 180

    position_error = math.sqrt(dx**2 + dy**2)

    # avoid division by zero
    if position_error < 1:
        return

    resonableTurn = 0.2  # deg per unit distance (your tuning param)

    # steering logic
    if abs(align_error) > resonableTurn * position_error:
        # too misaligned → just turn
        #Realign(state)
        veer(align_error / 45)
    else:
        # aligned enough → move forward with correction
        veer(align_error / 45)

    

def MoveToPoint(state:RobotState, point=None, next_Behaviour = None):
    destination = point
    if(next_Behaviour == None):
        nextBehavior = state.behaviour
    else:
        nextBehavior = next_Behaviour
    print("next behavior ", nextBehavior)
    def moveToPoint(state:RobotState):
        nonlocal nextBehavior
        nonlocal destination
        if(state.front_distance<20):
            wheels.stop()
            return nextBehavior

        if(np.linalg.norm(destination - state.position) < 5):
            wheels.stop()
            return nextBehavior
        else:
            to_target = destination - state.position
            angle = math.degrees(np.arctan2(to_target[1], to_target[0]))
            angle_error = ( angle -state.rotation  + 180) % 360 - 180
            if abs(angle_error)<45:
                MoveTo(state, destination)
                return moveToPoint
            else:
                return(SpinnTo(state, angle))
    return moveToPoint

                

    # wheels.speed = TURN_SPEED
    # wheels.spinn_right()
    # time.sleep(TURN_TIME)
    # wheels.stop()
def SpinnTo(state:RobotState, target_angle=None):
    angle = target_angle
    nextBehavior = state.behaviour
    def Realign(state:RobotState):
        nonlocal nextBehavior
        nonlocal angle
        error = ( angle -state.rotation  + 180) % 360 - 180
        if abs(error)<0.5:
            wheels.stop()
            print("next behavior ", nextBehavior)
            return nextBehavior
            
        else:
        
            
            
            wheels.speed =  SPEED
            if error<0 :
                wheels.spinn_right()
                state.direction = 0
            else:
                wheels.spinn_left()
                state.direction = 0
            return Realign
    return Realign
     
def TakePhoto(state:RobotState):
    
    filename = f"r{int(state.spinn.row):02d}c{int(state.spinn.stepCount):02d}.jpg"
    filepath = os.path.join(state.spinn.panoramafolder, filename)

    
    # Final capture
    subprocess.run([
        "fswebcam",
        "-r", "1920x1080",
        "--frames", "1",   # real improvement here
        "--skip", "30",
        "--no-banner",
        filepath
    ])
    
    print("Saved:", filename)
    
    
    
    


# def CaptureTest():
#     turns=0
#     while (turns<12):
#         UpDownTest()
#         CapturePanorama(state)
#         turns+=1

@dataclass
class SensorReading():
    time:float
    rotation:float
    left_distance:float
    front_distance:float
    right_distance:float
    score:float=0



def RealRun(state:RobotState): #Setup for real run, create folders and set camera settings
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    run_folder = os.path.join(base_folder, f"run_{run_id}")
    if len(sys.argv)>1:
        run_folder = os.path.join(base_folder,sys.argv[1])
        print(sys.argv[1])
    
    state.spinn.batchfolder = run_folder
    state.realRun = True
    state.spinn.lastPhotoSpot=(state.x,state.y)
    subprocess.run([
    "v4l2-ctl", "-d", "/dev/video0",
    "--set-fmt-video=width=1920,height=1080,pixelformat=MJPG",
    "-c", "auto_exposure=3",
    #"-c", "exposure_time_absolute=120",
    #"-c", "gain=0"
    ])
    
    # subprocess.run([
    #     "v4l2-ctl",
    #     "-d", "/dev/video0",
    #     "--set-fmt-video=width=1920,height=1080,pixelformat=MJPEG",
    #     "-c", "auto_exposure=1"
    # ], check=True)
def ReadLidar():

    while not lidar_queue.empty():
        scan = lidar_queue.get_nowait()
        print(scan)
        if(scan is not None):
            state.scan.readings.append(scan)
            if len(state.scan.readings)>10:
                state.scan.readings.pop(0)
        #print(scan.append)
    # closest_angle = None
    # closest_distance = float('inf')

    # for quality, angle, distance in scan:

    #     if distance < closest_distance:
    #         closest_distance = distance
    #         closest_angle = angle

    # print(closest_angle, closest_distance)

def ReadSensors(state:RobotState=state):
    #if(debug["sensors"]):
    if(len(state.readings)>0 and state.readings[-1].time- time.time())>3:
        print("Sensor queue delay: ", state.readings[-1].time - state.readings[0].time)
    state.world.obstructions = [
        (obs, timer)
        for obs, timer in state.world.obstructions
        if time.time() - timer <= 5
    ]
    while not sensor_queue.empty():
        left,front,right = sensor_queue.get()
        state.left_distance=left
        state.right_distance=right
        state.front_distance=front
        #only add readings when moving forward, sideways are unreliable and break average atm
        if(left>0): 
            state.Sensors.add_reading("left", left, state.x, state.y, state.rotation)
            if left<30:
                state.world.obstructions.append([state.Sensors.left_points[-1],time.time()])
                if len(state.world.obstructions) > 100:
                    state.world.obstructions.pop(0) 

        if(front>0): 
            state.Sensors.add_reading("front", front, state.x, state.y, state.rotation)
            if front<20:
                state.world.obstructions.append([state.Sensors.front_points[-1],time.time()])
                if len(state.world.obstructions) > 100:
                    state.world.obstructions.pop(0)
        if(right>0): 
            state.Sensors.add_reading("right", right, state.x, state.y, state.rotation)
            if right<30:
                state.world.obstructions.append([state.Sensors.right_points[-1],time.time()])
                if len(state.world.obstructions) > 100:
                    state.world.obstructions.pop(0)
        #state.scan.readings.append(SensorReading(time.time(),state.rotation,left,front,right))
        state.readings.append(SensorReading(time.time(),state.rotation,left,front,right))
        #ransac_lines = state.Sensors.ransac_line(state.Sensors.right_points+state.Sensors.left_points) if len(state.Sensors.right_points)+len(state.Sensors.left_points)>10 else None
        SendData(state)
        if len(state.readings)>10:
            state.readings.pop(0)
        if(debug["sensors"]):
            print(left, "|",front,"|",right, "|", time.time())
            #print(corridorAngle)
def SendData(state:RobotState):
        data = {
            "time": time.time(),
            "x":state.x,
            "y":state.y,
            "rotation":state.rotation,
            "left_distance":state.left_distance,
            "right_distance":state.right_distance,
            "front_distance":state.front_distance,
            "centerDirection": state.world.centerDirection.tolist(),
            "centerMean": state.world.centerMean.tolist(),
            
        }
        try:
            sock.sendto(json.dumps(data).encode(), (IP, PORT))
        
        except OSError as e:
            print(f"Network error: {e}")



lastSend = 0
def showOff(angle = None):
    lastSend = 0
    if time.time() - lastSend > 0.03:
        pitch = camera_servo.current_angle
        if(angle is not None):
            pitch = angle
        data = { "yaw" :state.rotation,
                    "pitch": pitch,
                    
                }
        print (data)
        try:
            sock.sendto(json.dumps(data).encode(), (IP, PORT))
            lastSend = time.time()
        
        except OSError as e:
                print(f"Network error: {e}")

def ReadGyro():
    global dt
    global lastSend
    raw = read_gyro_z()
    gyro_z = (raw - offset) / 131.0  # deg/sec
    state.rotation += gyro_z * dt
    state.npRotation = np.array([np.cos(math.radians(state.rotation)), np.sin(math.radians(state.rotation))])
    
    if debug["gryo"]:
        print(f"Rate: {gyro_z:6.2f} deg/s | Angle: {state.rotation:7.2f} deg")

def EstimateDistance(state):
        if 0<dt<1:
            v = Travel_Speed/100*(wheels.speedL + wheels.speedR)/2
            state.position[0] += v * math.cos(math.radians(state.rotation)) * dt*state.direction
            state.position[1] += v * math.sin(math.radians(state.rotation)) * dt*state.direction
            state.x, state.y = state.position[0], state.position[1]
            #print("Position: X: ", state.x, "Y: ",state.y)
            #sock.sendto(json.dumps([state.x,state.y]).encode(), (IP, PORT))
            #time.sleep(0.05)

def forwardCast(state, vector = None):
    if vector is None:
        vector = state.forwardVector()
    if len(state.world.obstructions)>0:
        for obs, timer in state.world.obstructions:
            if(np.dot(vector,(obs-state.position))>0 and squared_distance(state.position,obs)<50*50):
                if distancePointOnLine(obs,state.position,vector)<25:
                    return obs
         
    return None

def SteerCenter(state:RobotState):
    state.direction = 1
    if(state.realRun and ((state.lastPhotoSpot[0]-state.x)**2 + (state.lastPhotoSpot[1]-state.y)**2) > state.photoInterval**2):
        p = state.position
        c0 = state.world.centerMean
        d  = state.world.centerDirection
        d  = d / np.linalg.norm(d)  # make sure it's normalized
        
        proj = c0 + np.dot(p - c0, d) * d
        if(np.linalg.norm(proj-state.position)>10):
            return MoveToPoint(state, proj)
        center_angle = math.degrees(math.atan2(d[1], d[0]))

        angle_error = (state.rotation - center_angle + 180) % 360 - 180
        
        if(abs(angle_error)>5):
            return SpinnTo(state,center_angle)
        if((state.right_distance<100 or state.left_distance<100) and state.right_distance>0 and state.left_distance>0):
            midpoint = state.position+ (state.rightVector()*state.right_distance+state.leftVector()*state.left_distance)/2
            return MoveToPoint(state,midpoint)
        state.lastPhotoSpot=(state.x,state.y)
        wheels.stop()
        #state.mode = Mode.SPINNING
        return CapturePanorama
    

    if len(state.Sensors.right_points)>0: 
        if len(state.world.rightWall)==0 or state.world.rightWall[-1] !=state.Sensors.right_points[-1]:
            state.world.rightWall.append(state.Sensors.right_points[-1])
             
    if len(state.Sensors.left_points)>0: 
        if len(state.world.leftWall)==0 or state.world.leftWall[-1] !=state.Sensors.left_points[-1]:
            state.world.leftWall.append(state.Sensors.left_points[-1])
    # if(state.bashedHead>3):
    #     return Idle
    center_error =0
    p=0.5
    intCoeff=0.5
    d=.2
    kp_align=0.3
    derivative = 0
    integral = 0
    buffer_distance = 30
    tau = 0.15  # seconds
    #alpha = 1 - math.exp(-dt / tau)
    mean_alpha = 1*dt
    alpha = 0.5 * dt  # 0 = very stable, 1 = very reactive
    def furtherPoint():
        p = np.array([state.x, state.y])
        c0 = state.world.centerMean
        d = state.world.centerDirection
        d = d / np.linalg.norm(d)

        # robot_dir = np.array([
        #     math.cos(math.radians(state.rotation)),
        #     math.sin(math.radians(state.rotation))
        # ])

        if np.dot(d, state.world.centerDirection) < 0:
            d = -d

        proj = c0 + np.dot(p - c0, d) * d
        return proj + 100 * d

    def weight(rmse):
        return 1.0 / (rmse + 1e-6)
    l_angle, l_rmse, l_mean, l_direction = state.Sensors.fit_line_and_error(state.world.leftWall) or (None,None,None,None)#state.Sensors.get_leftWallAngle() or (None,None,None,None)
    r_angle, r_rmse, r_mean, r_direction = state.Sensors.fit_line_and_error(state.world.rightWall) or (None,None,None,None)
    if l_mean is not None and r_mean is not None:
        #print("rsme left ", l_rmse, " rmse right ", r_rmse)
        if(l_rmse<1 or r_rmse<1):
            
            w_l = weight(l_rmse)
            w_r = weight(r_rmse)
            bias = (w_l-w_r)/(w_l+w_r+1e-6)
            #confidence = abs(w_r - w_l) / (w_r + w_l + 1e-6)
            #offset = bias * confidence * buffer_distance * state.world.centerNormal
            offset = bias * buffer_distance * state.world.centerNormal
            # fix flipping
            if np.dot(l_direction, r_direction) < 0:
                r_direction = -r_direction
            if is_aligned(l_direction,r_direction,5) and l_rmse+r_rmse<2:
                pass
            else:
                if(is_aligned(l_direction,state.world.centerDirection,5)==False):
                    w_l = 0
                if(is_aligned(r_direction,state.world.centerDirection,5)==False):
                    w_r = 0
            if(w_l+w_r)>0:
                new_center_dir = w_l * l_direction + w_r * r_direction
                new_center_dir /= np.linalg.norm(new_center_dir)
                if np.dot(new_center_dir, state.world.centerDirection) < 0:
                    new_center_dir = -new_center_dir
                state.world.centerDirection = alpha * new_center_dir  + (1 - alpha) * state.world.centerDirection
            
            
            new_center_mean = (l_mean + r_mean) / (2) + offset


            # new_center_dir = (l_direction + r_direction) / 2
            #new_center_dir /= np.linalg.norm(new_center_dir)
            # #Flip direction if it points the wrong way
            

            state.world.centerMean= mean_alpha * new_center_mean + (1 - mean_alpha) * state.world.centerMean

            # IMPORTANT: re-normalize direction
            state.world.centerDirection = state.world.centerDirection / np.linalg.norm(state.world.centerDirection)
            state.world.centerNormal = np.array([-state.world.centerDirection[1], state.world.centerDirection[0]])  # perpendicular to line
    pos = np.array([state.x, state.y])
    delta = pos - state.world.centerMean
    error = delta @ state.world.centerNormal
    state.center_errors.append(error)
    if len(state.center_errors)>5:
        state.center_errors.pop(0)
    
    integral = sum(state.center_errors)/len(state.center_errors) if len(state.center_errors)>0 else 0    
    if len(state.center_errors) >= 2:
        derivative = (state.center_errors[-1] - state.center_errors[-2])
    else:
        derivative = 0
    targetPos = furtherPoint()
    to_target = targetPos - pos
    angle = math.degrees(np.arctan2(to_target[1], to_target[0]))
    angle_error = (angle-state.rotation  + 180) % 360 - 180
    if abs(angle_error)<45:
        MoveTo(state, targetPos)
        
    else:
        return(SpinnTo(state, angle))

    
    # if(0<state.front_distance<100):
    #     return(MoveToPoint(state, pos + 50*state.world.centerNormal if error>0 else pos - 50*state.world.centerNormal))
    if(0<state.front_distance<50):
        return Obstructed(state)
        delta = state.Sensors.front_points[0]-state.world.centerMean
        side = np.sign(np.dot(delta, state.world.centerNormal))
        offset = side * buffer_distance*2 * state.world.centerNormal
        return MoveToPoint(state,state.position-50*state.world.centerNormal*side)
        #return MoveToPoint(state,state.Sensors.front_points[-1]-50*state.world.centerNormal)
    if len(state.world.obstructions):
        for obs ,timer in state.world.obstructions:
            if(np.dot(state.forwardVector(),(obs-state.position))>0 and squared_distance(state.position,obs)<50*50):
                if distancePointOnLine(obs,state.position,state.forwardVector())<25:
                    return Obstructed(state)
    
    return SteerCenter
def followLine(mean, dir,dist = 100):
    p = state.position
    c0 = mean
    d = dir
    d = d / np.linalg.norm(d)

    # robot_dir = np.array([
    #     math.cos(math.radians(state.rotation)),
    #     math.sin(math.radians(state.rotation))
    # ])

    # if np.dot(d, state.world.centerDirection) < 0:
    #     d = -d

    proj = c0 + np.dot(p - c0, d) * d
    point = proj + dist * d
    MoveTo(state, point)
def distancePointOnLine(point, line_point, line_dir):
    P = point      # your point
    A = line_point    # a point on the line
    v = line_dir    # direction vector of the line

    AP = P - A

    # projection of AP onto v
    proj = (np.dot(AP, v) / np.dot(v, v)) * v

    # perpendicular component
    perp = AP - proj

    distance = np.linalg.norm(perp)
    return distance
def squared_distance(a, b):
    d = a - b
    return np.dot(d, d)

def angle_to_vector(deg):
    r = np.deg2rad(deg)
    return np.array([np.cos(r), np.sin(r)])

def is_aligned(v1, v2, tolerance_deg=15):
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    dot = np.dot(v1, v2)
    threshold = np.cos(np.deg2rad(tolerance_deg))

    return dot > threshold
def side_of(v, w):
    return v[0]*w[1] - v[1]*w[0]
        
def Obstructed(state: RobotState):
    startPoint = state.position
    
    obstructionPoints = np.array([])
    if side_of(state.world.centerMean, state.Sensors.front_points[-1]):
        side = "right"
    else:
        side = "left" 

    center = state.world.centerDirection
    left_normal  = np.array([-center[1], center[0]])
    right_normal = np.array([ center[1], -center[0]])
    buffer = 35
    safePoint = state.position.copy()
    checked_right = False
    checked_left = False
    #relevantSensor...
    def obstructed(state:RobotState):
        nonlocal safePoint
        nonlocal side
        
        nonlocal startPoint
        nonlocal buffer
        safeDistance = 100
        nonlocal left_normal
        nonlocal right_normal
        nonlocal checked_left
        nonlocal checked_right
        robot_dir = np.array([
            math.cos(math.radians(state.rotation)),
            math.sin(math.radians(state.rotation))
        ])
        
        
        obs = forwardCast(state)
        if (obs!= None):
            if(forwardCast(state, -state.forwardVector()) == None):
                #obs = forwardCast(state, left_normal)
                shiftAwayPoint = state.position+ state.position-obs
                return(MoveToPoint(state,shiftAwayPoint))


        if side == "left":
            if(is_aligned(robot_dir,left_normal,25)):
                followLine(startPoint, left_normal)
            else:
                
                return SpinnTo(state,math.degrees(math.atan2(left_normal[1],left_normal[0])))
            if(0<state.right_distance<safeDistance):
                safePoint = state.position.copy()
            if(distancePointOnLine(state.Sensors.right_points[-1], startPoint, left_normal)<buffer):
                startPoint -= state.world.centerDirection* buffer
        else:
            if(is_aligned(robot_dir,right_normal,25)):
                followLine(startPoint, right_normal)
            else:
                return SpinnTo(state,math.degrees(math.atan2(right_normal[1],right_normal[0])))
            if(0<state.left_distance<safeDistance):
                safePoint = state.position.copy()
            if(distancePointOnLine(state.Sensors.left_points[-1], startPoint, left_normal)<buffer):
                startPoint -= state.world.centerDirection* buffer
            
        if(np.linalg.norm(state.position-safePoint)> buffer):
            state.world.centerMean = state.position
            return SteerCenter#MoveToPoint(state,state.position+state.world.centerDirection*50,SteerCenter)
        if(0<state.front_distance<35 ):##Need to check for alignment/position, otherwise it will just resolve both
            if(side == "right" and is_aligned(robot_dir,right_normal,15)):
                
                checked_right = True
                side = "left"
            elif(side == "left" and is_aligned(robot_dir,left_normal,15)):
                
                checked_left = True
                side = "right"
                
        
        if(checked_left and checked_right):
            wheels.stop()
            return ManualDrive
        return obstructed
    return obstructed
    
            


def veer(error):
    wheels.forward()
    
    state.direction = 1
    
    steer = error
    if(debug["navigation"]):
        print("Error: ",error, " steer: ", steer)
        
    if(1<steer or steer<-1):
        print("Veer got",steer, "expected -1 to 1")
        steer = max(-1,min(steer,1))
    if(steer>0):
        
        wheels.speedL = int(SPEED-steer*SPEED)
        wheels.speedR = SPEED
    if(steer<0):
        wheels.speedL = SPEED
        wheels.speedR = int(SPEED+steer*SPEED)
    if(steer == 0):
        wheels.speedL = SPEED
        wheels.speedR = SPEED
def startWait(state:RobotState,waitTime:float):
    state.waitEndTime = time.time() + waitTime
    return Wait

def Wait(state:RobotState):
    if(time.time()<state.waitEndTime):
        wheels.stop()
        camera_servo.turn_straight()
        return Wait
    else:
        return state.lastbehaviour

def getch():
    """Read a single key press from the terminal"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


    return ch
## This function does not block the loop, but does not read the input until enter is
def get_key_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def Idle(state:RobotState):
    wheels.stop()
    camera_servo.turn_straight()
    return Idle
def ManualDrive(state:RobotState):
    ##print("Manual drive mode. Use WASD to drive, Q to quit.")
    
    key = get_key_nonblocking()
    if key == 'w':       # forward
        veer(0)
        # wheels.forward()
        # wheels.speed=SPEED
    elif key == 's':     # backward
        wheels.backward()
        
        state.direction = -1
        wheels.speed=SPEED
    elif key == 'a':     # turn left
        wheels.spinn_left()
        state.direction = 0
        wheels.speed = TURN_SPEED
    elif key == 'd':     # turn right
        wheels.spinn_right()
        state.direction = 0
        wheels.speed = TURN_SPEED
    elif key == ' ':     # stop
        wheels.stop()
        wheels.speedR=0
        wheels.speedL=0
        wheels.ready()
        camera_servo.turn_straight()
    elif key =="1": #try turning servo
        #state.targetAngle = (state.rotation + 90)
        #state.lastbehaviour = state.behaviour
        return Obstructed(state)
        #return MoveToPoint(state, np.array([state.x+random.uniform(-50,50), state.y+random.uniform(-50,50)]))
    #elif key =="2": #test Navigation
        #state.mode = Mode.ORIENTING
    elif key =="3": #testPhoto
        #state.mode = Mode.SPINNING
        RealRun(state)
        return CapturePanorama
    elif key =="4": #testPhoto
        #state.mode = Mode.SPINNING
        #RealRun(state)
        global showOffMode
        showOffMode = True
        return CapturePanoramaShowOff    
    elif key =="e":
        state.targetAngle = state.rotation
        state.world.centerDirection = np.array([math.cos(math.radians(state.rotation)), math.sin(math.radians(state.rotation))])
        state.world.centerMean = np.array([state.x, state.y])
        state.corridorAngle = state.rotation
        return SteerCenter
        
    elif key == 'q':     # quit
        wheels.stop()
        camera_servo.turn_straight()
        state.mode = Mode.IDLE
    return ManualDrive
    #else:
        
        ##wheels.stop()
        #camera_servo.turn_straight()
dt=0
prev_time=time.time()
US_Manager.start()
state.behaviour=ManualDrive
try:
    while True:
        behaviour = state.behaviour
        state.behaviour = state.behaviour(state)
        if(behaviour != state.behaviour):
            print("Switching behaviour from ", behaviour.__name__, " to ", state.behaviour.__name__ )
        if(showOffMode == True):
            showOff()   
        now = time.time()
        dt = now - prev_time if now - prev_time < 0.5 else 0.01
        prev_time = now
        ReadGyro()
        ReadSensors()
        EstimateDistance(state)
        if(lidar_connected):ReadLidar()
        # if min(read_line_follower())<30:
        #     # state.behaviour = ManualDrive
        #     # wheels.stop()
        #     print("Line lost, switching to manual control")
        
except KeyboardInterrupt:
    wheels.stop()
    camera_servo.turn_straight()
    US_Manager.stop()
finally:
    wheels.stop()
    wheels.speed=0
    camera_servo.turn_straight()
    US_Manager.stop()
    if(lidar_manager.connected):
        lidar_manager.stop()
