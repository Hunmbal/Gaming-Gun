import statistics
import smbus2 as smbus
from time import sleep
import math
import subprocess
import RPi.GPIO as GPIO  # type: ignore
import time

# New imports for GY-273
from smbus2 import SMBus

adb_command_start_time = time.time() + 3

# GPIO setup ------------------------------------------------------
GPIO.setmode(GPIO.BCM)
BUTTON_PIN = 23
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
switch_pin = 26
GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# MPU9250 Registers and their Addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

# HMC5883L (GY-273) Registers
HMC5883L_ADDRESS = 0x1E
CONFIG_REG_A = 0x00
CONFIG_REG_B = 0x01
MODE_REG = 0x02
DATA_REG_BEGIN = 0x03

# Initialize the bus and device addresses
bus = smbus.SMBus(1)
MPU9250_Address = 0x68
HMC5883L_Address = 0x1E

oP, oR, oY = 0, 0, 0
dP, dR, dY = 0, 0, 0
P, R, Y = 500, 500, 500
i,j = 0,0
sens = 1000
arr_dP = [0] * 10
arr_dY = [0] * 5

def MPU_Init():
    # Initialize MPU9250
    bus.write_byte_data(MPU9250_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU9250_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(MPU9250_Address, CONFIG, 0)
    bus.write_byte_data(MPU9250_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(MPU9250_Address, INT_ENABLE, 1)

def HMC5883L_Init():
    # Initialize HMC5883L (GY-273)
    bus.write_byte_data(HMC5883L_Address, CONFIG_REG_A, 0x70)
    bus.write_byte_data(HMC5883L_Address, CONFIG_REG_B, 0xA0)
    bus.write_byte_data(HMC5883L_Address, MODE_REG, 0x00)

def read_raw_data(addr):
    # Read raw data from MPU9250
    high = bus.read_byte_data(MPU9250_Address, addr)
    low = bus.read_byte_data(MPU9250_Address, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

def read_HMC5883L_data():
    # Read magnetometer data from HMC5883L
    data = bus.read_i2c_block_data(HMC5883L_Address, DATA_REG_BEGIN, 6)
    x = data[0] << 8 | data[1]
    z = data[2] << 8 | data[3]
    y = data[4] << 8 | data[5]
    if x > 32768: x = x - 65536
    if y > 32768: y = y - 65536
    if z > 32768: z = z - 65536
    return x, y, z

def calculate_pitch_roll_yaw(Ax, Ay, Az, Gy, Mx, My, Mz, prev_Y):
    P = 90 + math.atan(Ay / math.sqrt(Ax * Ax + Az * Az)) * (180 / math.pi)
    R = 180 * math.atan(Ax / math.sqrt(Ay * Ay + Az * Az)) / math.pi
    alpha = 0.98
    dt = 0.1
    # Use magnetometer data to calculate yaw
    yaw_mag = math.atan2(My, Mx) * (180 / math.pi)
    Y = alpha * (prev_Y + Gy * dt) + (1 - alpha) * yaw_mag
    return P, R, Y

def adb_swipe(end_x, end_y):
    # Format the swipe command
    cmd = f"adb shell input swipe 500 500 {end_x} {end_y} 1"
    # Execute the command
    subprocess.Popen(cmd, shell=True)
    #print(f"end_x: {end_x} \t end_y: {end_y} \n\n")
    

def dsp(arr1, arr2):
    t1 = statistics.mean(arr1)
    t2 = statistics.mean(arr2)
    return t1, t2

def isInPitchRange(x):
    if abs(x) > 0.15:
        return True
    else:
        return False

def isInYawRange(x):
    if abs(x) < 1.4 and abs(x) > 0.03 and abs(dR) > 0.09:
        return True
    else:
        return False

# Initialize sensors
MPU_Init()
HMC5883L_Init()
alpha = 0.86

while True:
    if GPIO.input(26) == GPIO.LOW:
        if GPIO.input(23) != GPIO.LOW:

            # Read Accelerometer raw values
            acc_x = read_raw_data(ACCEL_XOUT)
            acc_y = read_raw_data(ACCEL_YOUT)
            acc_z = read_raw_data(ACCEL_ZOUT)

            # Read Gyroscope raw values
            gyro_x = read_raw_data(GYRO_XOUT)
            gyro_y = read_raw_data(GYRO_YOUT)
            gyro_z = read_raw_data(GYRO_ZOUT)

            # Read Magnetometer raw values
            mag_x, mag_y, mag_z = read_HMC5883L_data()

            # Convert raw values to g, degrees/second, and gauss
            Ax = acc_x / 16384.0
            Ay = acc_y / 16384.0
            Az = acc_z / 16384.0
            Gx = gyro_x / 131.0
            Gy = gyro_y / 131.0
            Gz = gyro_z / 131.0
            Mx = mag_x * 0.92
            My = mag_y * 0.92
            Mz = mag_z * 0.92

            oP = P
            oR = R
            oY = Y

            #___________________________________________________________________________
            P, R, Y = calculate_pitch_roll_yaw(Ax, Ay, Az, Gy, Mx, My, Mz, oY)

            # Calculate yaw using magnetometer data
            yaw_mag = math.atan2(My, Mx)
            Y = alpha * (Y + 0.1 * Gz) + (1 - alpha) * yaw_mag
            #___________________________________________________________________________

            dP = oP - P
            dR = oR - R
            dY = oY - Y




            arr_dP[i] = dP
            arr_dY[j] = dY  # Store dY in the array
            i += 1
            j += 1
            if i == 10:
                i = 0
            if j == 5:
                j = 0

            dP, dY = dsp(arr_dP, arr_dY)  # Calculate mean of dP and dY

            end_x = 500 - (dY * sens * 8)
            end_y = 500 - (dP * sens)

            print(f"P: {P}\tY: {Y}")

            if time.time() > adb_command_start_time:  # start 3 sec late
                if isInPitchRange(dP) and isInYawRange(dY):
                    adb_swipe(end_x, end_y)
                elif isInPitchRange(dP):
                    adb_swipe(500, end_y)
                elif isInYawRange(dY):
                    adb_swipe(end_x, 500)

            sleep(0.1)
        else:
            print("Button was pushed!")
            cmd = "adb shell input tap 1500 650"
            subprocess.run(cmd, shell=True)
