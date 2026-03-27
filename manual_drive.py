import sys
import tty
import termios
import time
import random
from datetime import datetime
from picar import ultrasonic_module as UA4
import subprocess
import os
import smbus #for gyro

from picar import front_wheels, back_wheels  # PiCar-S library
from picar import Ultrasonic_Avoidance2 as UA2

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
print(picar.back_wheels.__file__)

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


#print("Tracking rotation...")


# Steering & speed parameters
STEER_ANGLE = 30  # degrees left/right
SPEED = 0        # speed 0-100 default 50
TURN_TIME = 1.6
#wheels.speed = SPEED
TURN_SPEED = 0 #default 30
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



def Roam():
    UA_F = UA2.Ultrasonic_Avoidance2(20)
    UA_L = UA4.Ultrasonic_Avoidance('D13', 'D10')
    UA_R = UA4.Ultrasonic_Avoidance('D14', 'D12')
    
    
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
    k = 0.1
    i=0.0000
    d=0.01
    angle = 0.0
    target_angle = 0.0
    prev_time = time.time()
    
    def HandleUltrasonicData(dist, lst):
        
        if 0<dist<1000:
            lst.append(dist)
            
        elif dist == -2:
            lst.append(200)
        else:
            print(f"Faulty reading {dist } from {'Right'if lst is distR else 'Left'}")
            print("lst is distL: ", lst is distL)
            lst.append(-3)  # Append a default value for faulty readings
 
    
        if(len(lst)>5):
                lst.pop(0)


    try:
        while True:
            raw = read_gyro_z()
            gyro_z = (raw - offset) / 131.0  # deg/sec

            now = time.time()
            dt = now - prev_time
            prev_time = now

            angle += gyro_z * dt

            print(f"Rate: {gyro_z:6.2f} deg/s | Angle: {angle:7.2f} deg")

            time.sleep(0.0001)


            # key = getch().lower()
            # if key == 'q':       # forward
            #     break
            distance_L =UA_L.get_distance()
            if(distance_L>0 and distance_L<1000):
                distL.append(distance_L)
            if(len(distL)>5):
                distL.pop(0)
            #time.sleep(0.05)
            distance = UA_F.get_distance()
            #time.sleep(0.05)
            distance_R = UA_R.get_distance()
            
            HandleUltrasonicData(distance_R,distR)
            HandleUltrasonicData(distance_L,distL)

            errors = []
            
            print(sum(distL)/len(distL) if distL else 0,"|",distance,"|",sum(distR)/len(distR) if distR else 0)
            #print("distance_F",distance)
            status = UA_F.less_than(threshold)
            trendL = 0
            for i in range(1, len(distL)):
                trendL += distL[i]-distL[i-1]

            trendR = 0
            for i in range(1, len(distR)):
                trendR += distR[i]-distR[i-1]

            #print("trend", trendL - trendR)    

            # if distance != -1:
            #     print('distance', distance, 'cm')
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
            if(distance != -1 and distance < front_clearance):
                obstructed = 1
            else:
                obstructed = 0  

            if(obstructed == 0):
                wheels.forward()
                wheels.speed = SPEED
                d_R = (sorted(distR)[len(distR)//2]) if distR else 0
                d_L = (sorted(distL)[len(distL)//2]) if distL else 0
                if(distR and distL):
                        
                    if d_R == -3:
                        confidence_R = 0
                    else:confidence_R = 1-distR.count (-3)/len(distR) if distR else 0
                    if(d_L == -3):
                        confidence_L = 0
                    else:confidence_L = 1-distL.count (-3)/len(distL) if distL else 0
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

                        print("Center_Offset ", target_angle)
                        #steer =(trendL-trendR)/100 
                        steer = k*(error) +i*integral + d*derivative #- (gyro_z)*i
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
                obs_distance = distance
                if (distance > front_clearance):
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
    if(steer<0):
        wheels.speedL = int(SPEED+steer*SPEED)
        wheels.speedR = SPEED
    if(steer>0):
        wheels.speedL = SPEED
        wheels.speedR = int(SPEED-steer*SPEED)
    
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

print("Manual drive mode. Use WASD to drive, Q to quit.")

try:
    while True:
        key = getch().lower()
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
            wheels.forward() 
            wheels.speedR = int(SPEED)
            wheels.speedL = int(0)
        elif key == 'q':     # quit
            wheels.stop()
            camera_servo.turn_straight()
            break
        else:
            ##wheels.stop()
            camera_servo.turn_straight()

except KeyboardInterrupt:
    wheels.stop()
    camera_servo.turn_straight()
