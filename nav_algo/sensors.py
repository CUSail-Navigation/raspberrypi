import nav_algo.low_level.nmea as nmea
import nav_algo.coordinates as coord
import nav_algo.low_level.SailSensors as SailSensors
from math import pi
import serial
import struct
import time
import numpy as np


class sensorData:
    def __init__(self, 
                coordinate_system=None, 
                mock_airmar=False,
                mock_anemometer=False):
        self.coordinate_system = coordinate_system
        self.mock_anemometer = mock_anemometer
        self.mock_airmar = mock_airmar

        # IMU
        self.pitch = 0
        self.roll = 0
        self.yaw = 0  # we read as wrt N, convert to wrt x-axis (E) (make sure 90 degrees is north)
        self.angular_velocity = 0
        self.prev_time_imu = None

        # anemometer
        self.wind_direction = 0  # wrt x-axis and noise removed
        self.wind_speed = 0  #don't need this, might add as an extra if there is time left
        self.anemomSMA = []  #helps remove noise from the anemometer reading
        self.relative_wind = 0 # wrt the boat (raw anemometer reading)

        # GPS
        self.fix = False
        self.latitude = 0.0
        self.longitude = 0.0
        self.velocity = None
        self.position = None
        self.prev_time_gps = None

        # Sensor objects
        
        if not self.mock_anemometer:
            self.anemometer = SailSensors.SailAnemometer(0)

        if not self.mock_airmar:
            self.AirMar = SailSensors.SailAirMar()

    def readAirMar(self):
        self.yaw = self.AirMar.readAirMarHeading() - 90 # offset 
        self.latitude = self.AirMar.readAirMarLatitude()
        self.longitude = self.AirMar.readAirMarLongitude()
        self.angular_velocity = self.AirMar.readAirMarROT()
        new_position = coord.Vector(self.coordinate_system, 
                                     self.latitude, self.longitude)
        cur_time = time.time()
        if self.prev_time_gps is not None:
                self.velocity = new_position.vectorSubtract(self.position)
                self.velocity.scale(1.0 / (cur_time - self.prev_time_gps))
        self.position = new_position
        self.prev_time_gps = cur_time
        
    # TODO: make sure this output is in polar coordinates.
    def readWindDirection(self):
        # returns the wind in polar coordinates
        rawData = self.anemometer.readAnemometerVoltage()
        self.wind_direction = (180 + 360 - rawData * 360 / 1720) % 360
        return

    def _addAverage(self, newValue):
        """Helper function that manages the SMA of the anemometer, this keeps the list at size =11 and returns the
        average of the list of ints. This function assumes that anemometer readings are taken semi-frequently
        parameter: newValue - int denoting number to be added to the """
        self.anemomSMA.append(newValue / 2)
        if (len(self.anemomSMA) > 1):
            for i in range(len(self.anemomSMA) - 1):
                self.anemomSMA[i] = self.anemomSMA[i] / 2
        if (len(self.anemomSMA) > 10):
            self.anemomSMA.pop(0)
        sum = 0
        for n in self.anemomSMA:
            sum = sum + n
        return sum

    def mockAirMar(self):
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 360 * np.random.rand()
        x = np.random.rand() * 200 - 100
        y = np.random.rand() * 200 - 100
        self.position = coord.Vector(x=x, y=y)

        vx = np.random.rand() - 0.5
        vy = np.random.rand() - 0.5
        self.velocity = coord.Vector(x=vx, y=vy)

    def mockAnemometer(self):
        self.wind_direction = 360 * np.random.rand()

    def readAll(self):
        if self.mock_anemometer:
            self.mockAnemometer()
        else:
            self.readWindDirection()

        if self.mock_airmar:
            self.mockAirMar()
        else:
            self.readAirMar()
