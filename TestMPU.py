import smbus
import time

bus = smbus.SMBus(1)  # J8 = I2C bus 1
MPU6050_ADDR = 0x68

# Registers
PWR_MGMT_1 = 0x6B
GYRO_ZOUT_H = 0x47
ACCEL_XOUT_H = 0x3B

# Wake up MPU6050
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

def read_accel():
    ax = read_word(ACCEL_XOUT_H)
    ay = read_word(ACCEL_XOUT_H + 2)
    az = read_word(ACCEL_XOUT_H + 4)
    return ax, ay, az

print("Reading MPU6050... Ctrl+C to stop")

try:
    while True:
        gz = read_gyro_z()
        ax, ay, az = read_accel()

        print(f"Gyro Z: {gz:6} | Accel: X={ax:6} Y={ay:6} Z={az:6}")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopped")