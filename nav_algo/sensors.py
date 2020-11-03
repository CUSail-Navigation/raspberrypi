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
        self.wind_direction = 0  # wrt x-axis and noise removed
        self.wind_speed = 0 #don't need this, might add as an extra if there is time left
        self.anemomSMA = [] #helps remove noise from the anemometer reading

        # GPS
        self.fix = False
        self.latitude = 0
        self.longitude = 0
        self.velocity = coord.Vector()

        #Sensor objects
        self.IMU = SailSensors.SailIMU()
        self.anemometer = SailSensors.SailAnemometer(0)

        #sensorData
        self.boat_direction = 0 # angle of the sail wrt north.
        self.sailAngleBoat = 0 #angle of the sail wrt to the boat.
        self.rawWind = 0


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
        self.yaw = 360+90 - eulerAngles[1]
        if self.yaw < 0:
            self.yaw +=360
        elif self.yaw > 360:
            self.yaw -=360
        self.boat_direction = self.yaw

        return

    def readWindDirection(self):
        rawData = self.anemometer.readAnemometerVoltage()
        self.rawWind = rawData
        if rawData >= 1601:
            rawAngle = 0
        else:
            rawAngle = rawData * 360 / 1600


        windWrtN = (rawAngle + self.sailAngleBoat) % 360
        windWrtN = (windWrtN + self.boat_direction) % 360
        self.wind_direction = self._addAverage(windWrtN)
        return

    def readGPS(self):
        # TODO update fix, lat, long, and velocity
        # use the NMEA parser
        pass

    """Helper function that manages the SMA of the anemometer, this keeps the list at size =11 and returns the
    average of the list of ints. This function assumes that anemometer readings are taken semi-frequently
    parameter: newValue - int denoting number to be added to the """
    def _addAverage(self,newValue):
        self.anemomSMA.append(newValue)/2
        if(len(self.anemomSMA > 1):
           self.anemomSMA[-2] = self.anemomSMA[-2]/2
        if(len(self.anemomSMA) > 10):
            self.anemomSMA.pop(0)
        sum = 0
        for n in self.anemomSMA:
            sum = sum + n
        return sum
