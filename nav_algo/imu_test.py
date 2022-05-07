import SailSensors
import struct
from math import pi
IMU = SailSensors.SailIMU()
pitch = 0
roll = 0
yaw = 0
boat_direction = 0

def readIMU():
    rawData = IMU.i2c_read_imu()
    eulerAngles = [0, 0, 0]
    #iterates through the list of raw data and converts int into a list of three floats
    for n in range(3):
        byteFloatList = rawData[4 * n:4 + 4 * n]
        eulerAngles[n] = struct.unpack(">f", bytes(byteFloatList))[0]
        eulerAngles[n] *= (180 / pi)

    pitch = eulerAngles[0]
    roll = eulerAngles[2]
    yaw = 360 + 90 - eulerAngles[1]
    if yaw < 0:
        yaw += 360
    elif yaw > 360:
        yaw -= 360
    boat_direction = yaw
    print(boat_direction)
    return
while(1):
    readIMU()