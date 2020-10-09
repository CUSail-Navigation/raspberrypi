import nmea as nmea
import coordinates as coord
import SailSensors
from math import pi

import struct

class sensorData:
    def __init__(self):
        # IMU
        self.pitch = 0
        self.roll = 0
        self.yaw = 0  # we read as wrt N, convert to wrt x-axis (E) (make sure 90 degrees is north)

        # anemometer
        self.wind_direction = 0  # wrt x-axis
        self.wind_speed = 0

        # GPS
        self.fix = False
        self.latitude = 0
        self.longitude = 0
        self.velocity = coord.Vector()

        self.IMU = SailSensors.SailIMU(SailSensors.IMU_ADDRESS)

    def readIMU(self):
        rawData = self.IMU.i2c_read_imu()
        eulerAngles = [0,0,0]
        #iterates through the list of raw data and converts int into a list of three floats
        for n in range(3):
            byteFloatList = rawData[4*n:4+4*n]
            eulerAngles[n] = struct.unpack(">f",bytes(byteFloatList))[0]
            eulerAngles[n] *= (180/pi)
            
        self.pitch = eulerAngles[0]
        self.roll = eulerAngles[2]
        self.yaw = eulerAngles[1]
        if self.yaw < 0:
            self.yaw +=360
        
        

        return

    def readWindDirection(self):
        # TODO remember we read this as wrt yaw, to get wrt x-axis, add yaw
        # and use coord.rangeAngle to get into the range [0, 360)
        pass

    def readGPS(self):
        # TODO update fix, lat, long, and velocity
        # use the NMEA parser
        pass
