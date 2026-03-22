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
fw = front_wheels.Front_Wheels(db='config')
bw = back_wheels.Back_Wheels(db='config')
bw.stop()
fw.turn_straight()
bw.speed = 0
fw.ready()
bw.ready()
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
SPEED = 50        # speed 0-100
TURN_TIME = 1.6
#bw.speed = SPEED
TURN_SPEED = 30
def UpDownTest():
    fw.turn_right()
    TakePhoto()
    time.sleep(1)
    fw.turn_straight()
    TakePhoto()
    time.sleep(1)
    fw.turn_left()
    TakePhoto()
    time.sleep(1)
    fw.turn_straight()
def SpinTest():
    bw.speed = TURN_SPEED
    bw.spin_right()
    time.sleep(TURN_TIME)
    bw.stop()
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
    
    distR=[]
    distL=[]

    k = 0.01
    i=0.005
    angle = 0.0
    target_angle = 0.0
    prev_time = time.time()
    try:
        while True:
            raw = read_gyro_z()
            gyro_z = (raw - offset) / 131.0  # deg/sec

            now = time.time()
            dt = now - prev_time
            prev_time = now

            angle += gyro_z * dt

            print(f"Rate: {gyro_z:6.2f} deg/s | Angle: {angle:7.2f} deg")

            time.sleep(0.01)


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
            if(distance_R>0 and distance_R<1000):
                distR.append(distance_R)
            if(len(distR)>5):
                distR.pop(0)
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
            
            if(distance>=40):
                bw.forward()
                bw.speed = SPEED
                
                if(distR and distL):
                    target_angle= (sorted(distR)[len(distR)//2] -sorted(distL)[len(distL)//2])/100
                    print("target ", target_angle)
                    #steer =(trendL-trendR)/100 
                    steer = (target_angle-angle)*k - (gyro_z)*i
                    steer = max(-1,min(steer,1))
                    #print("filterd ", steer)
                    print("steer ", steer)
                    veer(-steer)
                #else:veer(-1)


                
            # elif(25<=distance<=40):
            #     bw.spin_right()
            #     bw.speed = TURN_SPEED
            else:
                bw.backward()
                bw.speed = TURN_SPEED
                time.sleep(1)
    except Exception:
        bw.stop()
        fw.turn_straight()
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
        bw.speedL = int(SPEED+steer*SPEED)
        bw.speedR = SPEED
    if(steer>0):
        bw.speedL = SPEED
        bw.speedR = int(SPEED-steer*SPEED)
    
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
            bw.forward()
            bw.speed=SPEED
        elif key == 's':     # backward
            bw.backward()
            bw.speed=SPEED
        elif key == 'a':     # turn left
            bw.spin_left()
            bw.speed = TURN_SPEED
        elif key == 'd':     # turn right
            bw.spin_right()
            bw.speed = TURN_SPEED
        elif key == ' ':     # stop
            bw.stop()
            bw.ready()
            fw.turn_straight()
        elif key =="1": #try turning servo
            CaptureTest()
        elif key =="2": #test Navigation
            Roam()
        elif key =="3": #testPhoto
            TakePhoto()
        elif key =="e":
            bw.forward() 
            bw.speedR = int(SPEED)
            bw.speedL = int(0)
        elif key == 'q':     # quit
            bw.stop()
            fw.turn_straight()
            break
        else:
            ##bw.stop()
            fw.turn_straight()

except KeyboardInterrupt:
    bw.stop()
    fw.turn_straight()
