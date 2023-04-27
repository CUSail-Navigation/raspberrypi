import SailSensors
import struct
from math import pi
import time

IMU = SailSensors.SailIMU()
pitch = 0
roll = 0
yaw = 0
boat_direction = 0

sailAngleBoat  = -180
boat_direction = 0
anemomSMA=[]
anemometer = SailSensors.SailAnemometer(0)

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

    return yaw
    

def readWindDirection():
        
        rawData = anemometer.readAnemometerVoltage()
        """print(rawData)"""
        rawWind = rawData
        rawAngle = (360 - rawData * 360 / 1700) + 180

        windWrtN = (rawAngle + sailAngleBoat)
        windWrtN = (windWrtN + boat_direction + 270) % 360
        wind_direction = _addAverage(windWrtN)
        print(wind_direction)
        return
    
    
def _addAverage(newValue):
        """Helper function that manages the SMA of the anemometer, this keeps the list at size =11 and returns the
        average of the list of ints. This function assumes that anemometer readings are taken semi-frequently
        parameter: newValue - int denoting number to be added to the """
        anemomSMA.append(newValue / 2)
        if (len(anemomSMA) > 1):
            for i in range(len(anemomSMA)-1):
                anemomSMA[i] = anemomSMA[i] / 2
        if (len(anemomSMA) > 10):
            anemomSMA.pop(0)
        sum = 0
        for n in anemomSMA:
            sum = sum + n
        return sum

while (1):
    yaw = readIMU()
    boat_direction = yaw
    print("yaw: " + str(yaw))
    readWindDirection()
    time.sleep(1)