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
                mock_gps=True,
                mock_imu=True,
                mock_anemometer=True):
        self.coordinate_system = coordinate_system
        self.mock_gps = mock_gps
        self.mock_imu = mock_imu
        self.mock_anemometer = mock_anemometer
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
        if not self.mock_imu:
            self.IMU = SailSensors.SailIMU()
        
        if not self.mock_anemometer:
            self.anemometer = SailSensors.SailAnemometer(0)
        
        if not self.mock_gps:
            self.gps_serial_port = serial.Serial(port='/dev/ttyAMA3',
                                                 baudrate=9600,
                                                 timeout=1)
            
        # self.AirMar = SailSensors.SailAirMar()

    def readAirMar(self):
        self.yaw = self.AirMar.readAirMarHeading()
        # TODO: Add an indicator in AirMar class for GPS fix
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
        

    def readIMU(self):
        rawData = self.IMU.i2c_read_imu()
        eulerAngles = [0, 0, 0]
        prev_yaw = self.yaw 

        #iterates through the list of raw data and converts int into a list of three floats
        for n in range(3):
            byteFloatList = rawData[4 * n:4 + 4 * n]
            eulerAngles[n] = struct.unpack(">f", bytes(byteFloatList))[0]
            eulerAngles[n] *= (180 / pi)

        self.pitch = eulerAngles[0]
        self.roll = eulerAngles[2]
        self.yaw = 360 + 90 - eulerAngles[1]
        self.yaw = self.yaw % 360

        cur_time = time.time()
        if self.prev_time_imu is not None:
            angle_diff = 180 - abs(abs(prev_yaw - self.yaw) - 180)
            self.angular_velocity = angle_diff / (cur_time - self.prev_time_imu)

        self.prev_time_imu = cur_time
        return

    def readWindDirection(self):
        # returns the wind in polar coordinates
        rawData = self.anemometer.readAnemometerVoltage()
        self.wind_direction = (-294 + 90 - rawData * 360 / 1720) % 360
        return

    def readGPS(self):
        # use the NMEA parser
        # TODO you may want to just leave this open
        if(self.gps_serial_port.is_open):
            self.gps_serial_port.close()
        self.gps_serial_port.open()
        self.gps_serial_port.reset_input_buffer()
        for _ in range(5):
            try:
                line = self.gps_serial_port.readline().decode('utf-8')
                #print(line)
            except:
                line = ''
            nmea_data = nmea.NMEA(line)
            if nmea_data.status:
                self.fix = True
                self.latitude = nmea_data.latitude
                self.longitude = nmea_data.longitude
                new_position = coord.Vector(self.coordinate_system,
                                            self.latitude, self.longitude)
                cur_time = time.time()

                if self.prev_time_gps is None:
                    self.position = new_position
                    self.prev_time_gps = cur_time
                else:
                    self.velocity = new_position.vectorSubtract(self.position)
                    self.velocity.scale(1.0 / (cur_time - self.prev_time_gps))
                    self.position = new_position
                    self.prev_time_gps = cur_time
                    
        self.gps_serial_port.close()

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

    def mockIMU(self):
        # self.pitch = 0.0
        # self.roll = 0.0
        # self.yaw = 360 * np.random.rand()
        pass
    
    def mockGPS(self):
        x = np.random.rand() * 200 - 100
        y = np.random.rand() * 200 - 100
        self.position = coord.Vector(x=x, y=y)

        vx = np.random.rand() - 0.5
        vy = np.random.rand() - 0.5
        self.velocity = coord.Vector(x=vx, y=vy)

    def mockAnemometer(self):
        self.wind_direction = 360 * np.random.rand()

    def readAll(self):
        if self.mock_imu:
            self.mockIMU()
        else:
            self.readIMU()

        if self.mock_anemometer:
            self.mockAnemometer()
        else:
            self.readWindDirection()

        if self.mock_gps:
            self.mockGPS()
        else:
            self.readGPS()

        # if self.AirMar:
               # self.readAirMar()
