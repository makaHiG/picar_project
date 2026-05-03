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

import numpy as np
# from . import ultrasonic_manager
# from ultrasonic_manager import UltrasonicManager
# import ultrasonic_module as UA4
from .ultrasonic_manager import UltrasonicManager
#from . import ultrasonic_module as UA4
from .state import RobotState,Mode,ScanState,SpinnState

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
# Initialize ultrasonic sensors
sensor_queue = Queue()
US_Manager = UltrasonicManager(20, (16,12), (26,19), sensor_queue)

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

bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
# Gyro functions
def read_word(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

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
    "sensors": True,
    "gryo": False,
    "navigation": False
}

#print("Tracking rotation...")

# Steering & speed parameters
STEER_ANGLE = 30  # degrees left/right
SPEED = 100       # speed 0-100 default 50
TURN_TIME = 1.6
#wheels.speed = SPEED
TURN_SPEED = 100#default 30
Travel_Speed = 44*3.14/15 #Speed from test,cm/s



#Take Photos, angles defined in state    
def PhotoCollumn(state:RobotState=state):
    for i in range(len(state.rowAngles)):
        angle = state.rowAngles[i]
        camera_servo.turn(angle)
        time.sleep(1)
        state.spinn.row=i
        if(state.realRun):
            TakePhoto(state)
    if(state.spinn.stepCount == 0):
        state.spinn.row="zenith"
        camera_servo.turn(180)
        time.sleep(1)
        if(state.realRun):
            TakePhoto(state)

def CapturePanorama(state:RobotState):
    spinn = state.spinn
    spinn: SpinnState
    if(spinn.active == False):
        if(state.realRun == True):
            spinn.panoramafolder = os.path.join(spinn.batchfolder, "panorama"+str(spinn.panoramacounter))
            # os.path.expanduser("~/photos")
            os.makedirs(spinn.panoramafolder, exist_ok=True)
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
        mod = error /3
        wheels.speed = int(min(100,max(25,TURN_SPEED*mod)))
        if error<0 :
            wheels.spinn_right()
            state.direction = 0
        else:
            wheels.spinn_left()
            state.direction = 0
    return CapturePanorama

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

    

def MoveToPoint(state:RobotState, point=None):
    destination = point
    nextBehavior = state.behaviour
    print("next behavior ", nextBehavior)
    def moveToPoint(state:RobotState):
        nonlocal nextBehavior
        nonlocal destination
        if(state.front_distance<20):
            wheels.stop()
            return nextBehavior

        if(np.linalg.norm(destination - state.position) < 1):
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
    
    filename = f"r{state.spinn.row}c{state.spinn.stepCount}.jpg"
    filepath = os.path.join(state.spinn.panoramafolder, filename)

    
    # Final capture
    subprocess.run([
        "fswebcam",
        "-r", "1920x1080",
        "--frames", "1",   # real improvement here
        "--skip", "10",
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
    state.spinn.batchfolder = run_folder
    state.realRun = True
    state.spinn.lastPhotoSpot=(state.x,state.y)
    subprocess.run([
    "v4l2-ctl", "-d", "/dev/video0",
    "--set-fmt-video=width=1920,height=1080,pixelformat=MJPG",
    "-c", "auto_exposure=3",
    #"-c", "exposure_time_absolute=120",
    "-c", "gain=0"
    ])
    
    # subprocess.run([
    #     "v4l2-ctl",
    #     "-d", "/dev/video0",
    #     "--set-fmt-video=width=1920,height=1080,pixelformat=MJPEG",
    #     "-c", "auto_exposure=1"
    # ], check=True)

def ReadSensors(state:RobotState=state):
    while not sensor_queue.empty():
        left,front,right = sensor_queue.get()
        state.left_distance=left
        state.right_distance=right
        state.front_distance=front
        if(state.direction == 1): #only add readings when moving forward, sideways are unreliable and break average atm
            if(left>0): state.Sensors.add_reading("left", left, state.x, state.y, state.rotation)

            if(front>0): state.Sensors.add_reading("front", front, state.x, state.y, state.rotation)

            if(right>0): state.Sensors.add_reading("right", right, state.x, state.y, state.rotation)

        #state.scan.readings.append(SensorReading(time.time(),state.rotation,left,front,right))
        state.readings.append(SensorReading(time.time(),state.rotation,left,front,right))
        #ransac_lines = state.Sensors.ransac_line(state.Sensors.right_points+state.Sensors.left_points) if len(state.Sensors.right_points)+len(state.Sensors.left_points)>10 else None
        data = {
            "time": time.time(),
            "x":state.x,
            "y":state.y,
            "rotation":state.rotation,
            "left_distance":left,
            "right_distance":right,
            "front_distance":front,
            "centerDirection": state.world.centerDirection.tolist(),
            "centerMean": state.world.centerMean.tolist(),
            #"ransacLines": ransac_lines.tolist() if ransac_lines is not None else None 
        }
        try:
            sock.sendto(json.dumps(data).encode(), (IP, PORT))
    
        except OSError as e:
            print(f"Network error: {e}")

        if len(state.readings)>10:
            state.readings.pop(0)
        if(debug["sensors"]):
            print(left, "|",front,"|",right)
            #print(corridorAngle)
def ReadGyro():
    raw = read_gyro_z()
    gyro_z = (raw - offset) / 131.0  # deg/sec
    state.rotation += gyro_z * dt

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

def OrientationSpinn(state=state): ## Deprecated, not used anymore
    scan=state.scan
    scan:ScanState
    if(scan.active == False):
        wheels.spinn_left()
        wheels.speed = TURN_SPEED
        scan.startRotation=state.rotation
        scan.readings.clear()
        scan.active = True
    
    
    lowestLeft=None
    lowestRight=None
    lowestAdded=None
    if(state.rotation>scan.startRotation+360):
        singleReadings=[]
        for reading in scan.readings:
            reading:SensorReading
            if reading.left_distance > 0:
                 if lowestLeft == None or reading.left_distance<lowestLeft.left_distance:
                    lowestLeft = reading
            if reading.right_distance > 0: 
                if(lowestRight == None or reading.right_distance<lowestRight.right_distance):
                    lowestRight = reading
            if reading.left_distance > 0 and reading.right_distance > 0:
                if lowestAdded == None or reading.right_distance+reading.left_distance<lowestAdded.right_distance+lowestAdded.left_distance:
                    lowestAdded = reading
            singleReadings.append([(reading.rotation - 90) % 360, reading.right_distance])
            singleReadings.append([reading.rotation % 360, reading.front_distance])
            singleReadings.append([(reading.rotation + 90) % 360, reading.left_distance])   
            #sock.sendto(json.dumps(singleReadings).encode(), (IP, PORT))
            
        ## ADD a Check values against curves to check if it is likely to be valid.
        ## ADD Check that front is clear
        if lowestAdded is not None:
            diff = (state.corridorAngle - lowestAdded.rotation + 180) % 360 - 180

            if abs(diff) > 90:
                state.corridorAngle = (lowestAdded.rotation + 180) % 360
            else:
                state.corridorAngle = lowestAdded.rotation
            
        if(debug["navigation"]):
            print("coordiorAngelApriximated at ",lowestAdded.rotation)
            scan.active=False
        
        wheels.stop()    
        state.mode=Mode.DIRECTIONAL_MOVE
                   

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
        
        if(abs(angle_error)>10):
            return SpinnTo(state,center_angle)
        state.lastPhotoSpot=(state.x,state.y)
        wheels.stop()
        #state.mode = Mode.SPINNING
        return CapturePanorama
    
    if(state.bashedHead>3):
        return Idle
    center_error =0
    p=0.5
    intCoeff=0.5
    d=.2
    kp_align=0.3
    derivative = 0
    integral = 0
    buffer_distance = 30
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
    l_angle, l_rmse, l_mean, l_direction = state.Sensors.get_leftWallAngle() or (None,None,None,None)
    r_angle, r_rmse, r_mean, r_direction = state.Sensors.get_rightWallAngle() or (None,None,None,None)
    if l_mean is not None and r_mean is not None:
        #print("rsme left ", l_rmse, " rmse right ", r_rmse)
        if(l_rmse<1 or r_rmse<1):
            
            w_l = weight(l_rmse)
            w_r = weight(r_rmse)
            bias = (w_l-w_r)/(w_l+w_r+1e-6)
            confidence = abs(w_r - w_l) / (w_r + w_l + 1e-6)
            offset = bias * confidence * buffer_distance * state.world.centerNormal
            # fix flipping
            if np.dot(l_direction, r_direction) < 0:
                r_direction = -r_direction
            if(np.dot(np.linalg.norm(l_direction), np.linalg.norm(r_direction)))<0.1 and l_rmse+r_rmse<2:
                pass
            else:
                if(abs( np.dot(np.linalg.norm(l_direction), np.linalg.norm(state.world.centerDirection)))> 0.2):
                    w_l = 0
                if(abs( np.dot(np.linalg.norm(r_direction), np.linalg.norm(state.world.centerDirection)))> 0.2):
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
            alpha = 0.15  # 0 = very stable, 1 = very reactive

            state.world.centerMean= alpha * new_center_mean + (1 - alpha) * state.world.centerMean

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
                delta = state.Sensors.front_points[0]-state.world.centerMean
                side = np.sign(np.dot(delta, state.world.centerNormal))
                offset = side * buffer_distance*2 * state.world.centerNormal
                return MoveToPoint(state,state.position-50*state.world.centerNormal*side)
                #return MoveToPoint(state,state.Sensors.front_points[-1]-50*state.world.centerNormal)
    if(0<state.front_distance<20):
        return MoveToPoint(state, (state.position - 30*state.world.centerNormal))(state)
        
        state.bashedHead+=1
        wheels.backward()
        wheels.speed = TURN_SPEED
        time.sleep(1)
        #state.mode = Mode.ORIENTING

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
    proj + dist * d
    MoveTo(proj)
    
def Obstructed(state: RobotState):
    startPoint = state.position
    # followLine(startPoint, state.world.centerNormal)
    # obstructionPoints = np.array([])
    # buffer = 50
    # robot_dir = np.array([
    #     math.cos(math.radians(state.rotation)),
    #     math.sin(math.radians(state.rotation))
    # ])
    # if(relevantSensor<safe):
    #     safePoint = state.position
    # else:
    #     if(np.linalg.norm(state.position-safePoint)> buffer):
    #         return SteerCenter
    # if(state.front_distance<20 and np.array([math.cos(state.rotation),


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

        return MoveToPoint(state, np.array([state.x+random.uniform(-50,50), state.y+random.uniform(-50,50)]))
    #elif key =="2": #test Navigation
        #state.mode = Mode.ORIENTING
    elif key =="3": #testPhoto
        #state.mode = Mode.SPINNING
        RealRun(state)
        return CapturePanorama
        
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
        #sock.sendto(b"Hello", ("255.255.255.255", 5005))
        # if(get_key_nonblocking()=="m"):
            #     state.mode = Mode.MANUAL
            
        now = time.time()
        dt = now - prev_time if now - prev_time < 0.5 else 0.01
        prev_time = now
        ReadGyro()
        ReadSensors()
        EstimateDistance(state)
        
        # if(state.mode == Mode.MANUAL):
        #     ManualDrive(state)
        # if(state.mode == Mode.DIRECTIONAL_MOVE):
        #     SteerCenter(state)
        # if(state.mode == Mode.ORIENTING):
        #     OrientationSpinn(state)
        # if(state.mode == Mode.SPINNING):
        #     CapturePanorama(state)
except KeyboardInterrupt:
    wheels.stop()
    camera_servo.turn_straight()
    US_Manager.stop()
finally:
    wheels.stop()
    wheels.speed=0
    camera_servo.turn_straight()
    US_Manager.stop()
