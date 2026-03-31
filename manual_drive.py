import select
import sys
import tty
import termios
import time
import math
from queue import Queue
import random
from datetime import datetime
# from . import ultrasonic_manager
# from ultrasonic_manager import UltrasonicManager
# import ultrasonic_module as UA4
from .ultrasonic_manager import UltrasonicManager
from . import ultrasonic_module as UA4
from .ultrasonic_avoidance_3pin import Ultrasonic_Avoidance2 as UA2
from .state import RobotState,Mode

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

# Initialize ultrasonic sensors
sensor_queue = Queue()
UA_F = UA2(20)
UA_L = UA4.Ultrasonic_Avoidance('D13', 'D10')
UA_R = UA4.Ultrasonic_Avoidance('D14', 'D12')
US_Manager =UltrasonicManager(UA_F, UA_L, UA_R,sensor_queue)

# Gyro setup
bus = smbus.SMBus(1)
MPU6050_ADDR = 0x68

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
    "gryo": True,
    "navigation": True
}
#print("Tracking rotation...")


# Steering & speed parameters
STEER_ANGLE = 30  # degrees left/right
SPEED = 100       # speed 0-100 default 50
TURN_TIME = 1.6
#wheels.speed = SPEED
TURN_SPEED = 100 #default 30
Travel_Speed = 44*3.14/15 #Speed from test,cm/s

def UpDownTest():
    camera_servo.turn_right()
    TakePhoto()
    time.sleep(1)
    camera_servo.turn_straight()
    TakePhoto()
    time.sleep(1)
    camera_servo.turn_left()
    TakePhoto()
    time.sleep(1)
    camera_servo.turn_straight()
def SpinTest():
    
    wheels.speed = TURN_SPEED
    wheels.spin_right()
    time.sleep(TURN_TIME)
    wheels.stop()
def CaptureTest():
    turns=0
    while (turns<12):
        UpDownTest()
        SpinTest()
        turns+=1



def ReadSensors():
    while not sensor_queue.empty():
        left,front,right = sensor_queue.get()
        state.left_distance=left
        state.right_distance=right
        state.front_distance=front
        if(debug["sensors"]):
            print(left, "|",front,"|",right)
def ReadGyro():
    raw = read_gyro_z()
    gyro_z = (raw - offset) / 131.0  # deg/sec

    

    state.rotation += gyro_z * dt

    if debug["gryo"]:
        print(f"Rate: {gyro_z:6.2f} deg/s | Angle: {state.rotation:7.2f} deg")



def SteerCenter():
    if(state.right_distance>0 and state.left_distance>0):
        offset = (state.right_distance - state.left_distance)/(state.left_distance+state.right_distance)
        veer(offset)
        print("Offset= ", offset)
    else:
        veer(0)
        print("Go Straight")
    if(0<state.front_distance<50):
        wheels.backward()
        wheels.speed = TURN_SPEED
        time.sleep(1)
    

def Roam():
    
    
    
    threshold = 10
    cor_angle = 0
    distR=[]
    confidence_R =1.0
    distL=[]
    confidence_L =1.0
    obstructed = 0
    obs_distance = 0
    clear_angle = 0
    front_clearance = 30 #distance considered clear in front of the car
    side_clearance = 20 #clearance needed sideways
    k = 1
    i=0.0000
    d=0.00
    angle = 0.0
    target_angle = 0.0
    prev_time = time.time() ##
    distance_x=0
    distance_y=0
    def DriveStraight():
        veer(angle-cor_angle)
    def EstimateDistance(angle): ##Not functional 
        # now = time.time()
        # dt = now - prev_time
        prev_time = now
        distance_x+= math.sin(angle)*Travel_Speed*dt*SPEED
        distance_y+= math.cos(angle)*Travel_Speed*dt*SPEED 
        if debug["cordinates"]:
            print("X:",distance_x,"Y:",distance_y) 

    try:
        while True:
            time.sleep(0.01)
            raw = read_gyro_z()
            gyro_z = (raw - offset) / 131.0  # deg/sec

            now = time.time()
            dt = now - prev_time
            prev_time = now

            angle += gyro_z * dt

            if debug["gryo"]:
                print(f"Rate: {gyro_z:6.2f} deg/s | Angle: {angle:7.2f} deg")

            time.sleep(0.001)


            # key = getch().lower()
            # if key == 'q':       # forward
            #     break
            distance_L =US_Manager.left_distance
            # if(distance_L>0 and distance_L<1000):
            #     distL.append(distance_L)
            # if(len(distL)>5):
            #     distL.pop(0)
            #time.sleep(0.05)
            distance_F = US_Manager.front_distance
            #time.sleep(0.05)
            distance_R = US_Manager.right_distance


            errors = []
            
            #print("distance_F",distance)
            # trendL = 0
            # for i in range(1, len(distL)):
            #     trendL += distL[i]-distL[i-1]

            # trendR = 0
            # for i in range(1, len(distR)):
            #     trendR += distR[i]-distR[i-1]

            #print("trend", trendL - trendR)    

            # if distance_F != -1:
            #     print('distance_F', distance_F, 'cm')
            #     time.sleep(0.2)
            # else:
            #     print(False)
            # if status == 1:
            #     print("Less than %d" % threshold)
            # elif status == 0:
            #     print("Over %d" % threshold)
            # else:
            #     print("Read distance error2.")
            # print(status)
            if(distance_F != -1 and distance_F < front_clearance):
                obstructed = 1
            else:
                obstructed = 0  

            if(obstructed == 0):
                wheels.forward()
                wheels.speed = SPEED
                d_R = distance_R
                d_L = distance_L
                
                    
                if d_R == -3:
                    confidence_R = 0
                else: confidence_R = 1
                #else:confidence_R = 1-distR.count (-3)/len(distR) if distR else 0
                if(d_L == -3):
                    confidence_L = 0
                else: confidence_L = 1
                #else:confidence_L = 1-distL.count (-3)/len(distL) if distL else 0
                target_angle = cor_angle
                steer = 0
                if(confidence_L>0 and confidence_R>0):
                    target_angle =  (d_L - d_R) / (d_R + d_L) if (d_R + d_L) != 0 else 0
                    error = (target_angle*90-angle)/90
                    errors.append(error)
                    integral = sum(errors)
                    if(len(errors)>5):
                        errors.pop(0)
                    derivative = 0
                    for i in range(1, len(errors)):
                        derivative += errors[i] - errors[i-1]

                    #steer =(trendL-trendR)/100 
                    steer = k*(error) +i*integral + d*derivative #- (gyro_z)*i
                    if debug["navigation"]:
                        print("Center_Offset ", target_angle)
                        print("steer ", steer)
                    steer = max(-1,min(steer,1))
                    #print("filterd ", steer)
                else:
                    steer = (target_angle-angle)/90
                    steer = max(-1,min(steer,1))
                veer(-steer)
                
                #else:veer(-1)


                
            # elif(25<=distance<=40):
            #     wheels.spin_right()
            #     wheels.speed = TURN_SPEED
            
            elif(obstructed == 1):
                obs_distance = distance_F
                if (distance_F > front_clearance):
                    print("free")
                wheels.backward()
                wheels.speed = TURN_SPEED
                time.sleep(1)

    except Exception:
        wheels.stop()
        camera_servo.turn_straight()
        raise
            
def TakePhoto():
    folder = os.path.expanduser("~/photos")
    os.makedirs(folder, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"{folder}/photo_{timestamp}.jpg"

    subprocess.run([
        "fswebcam",
        "-r", "1920x1080",   # resolution
        "--frames", "10",    # warm-up frames for exposure
        "--no-banner",
        filename
    ])
    

    print("Saved:", filename)

def veer(steer):
    wheels.forward()
    if(1<steer or steer<-1):
        print("Veer got",steer, "expected -1 to 1")
        steer = max(-1,min(steer,1))
    if(steer<0):
        
        wheels.speedL = int(SPEED+steer*SPEED)
        wheels.speedR = SPEED
    if(steer>0):
        wheels.speedL = SPEED
        wheels.speedR = int(SPEED-steer*SPEED)
    if(steer == 0):
        wheels.speedL = SPEED
        wheels.speedR = SPEED
    
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
def ManualDrive(state):
    print("Manual drive mode. Use WASD to drive, Q to quit.")
    
    key = getch()
    if key == 'w':       # forward
        wheels.forward()
        wheels.speed=SPEED
    elif key == 's':     # backward
        wheels.backward()
        wheels.speed=SPEED
    elif key == 'a':     # turn left
        wheels.spin_left()
        wheels.speed = TURN_SPEED
    elif key == 'd':     # turn right
        wheels.spin_right()
        wheels.speed = TURN_SPEED
    elif key == ' ':     # stop
        wheels.stop()
        wheels.ready()
        camera_servo.turn_straight()
    elif key =="1": #try turning servo
        CaptureTest()
    elif key =="2": #test Navigation
        Roam()
    elif key =="3": #testPhoto
        TakePhoto()
    elif key =="e":
        state.mode=Mode.DIRECTIONAL_MOVE
    elif key == 'q':     # quit
        wheels.stop()
        camera_servo.turn_straight()
        state.mode = Mode.IDLE
    else:
        ##wheels.stop()
        camera_servo.turn_straight()
dt=0
prev_time=time.time()
US_Manager.start()
state.mode=Mode.MANUAL
try:
    while True:

        now = time.time()
        dt = now - prev_time
        prev_time = now
        ReadGyro()
        ReadSensors()
        if(state.mode == Mode.MANUAL):
            ManualDrive(state)
        if(state.mode == Mode.DIRECTIONAL_MOVE):
            SteerCenter()
except KeyboardInterrupt:
    wheels.stop()
    camera_servo.turn_straight()
