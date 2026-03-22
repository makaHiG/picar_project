import smbus
import time

bus = smbus.SMBus(1)
MPU6050_ADDR = 0x68

PWR_MGMT_1 = 0x6B
GYRO_ZOUT_H = 0x47

bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

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
print("Offset:", offset)

# --- Integration ---
angle = 0.0
prev_time = time.time()

print("Tracking rotation...")

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

except KeyboardInterrupt:
    print("\nStopped")