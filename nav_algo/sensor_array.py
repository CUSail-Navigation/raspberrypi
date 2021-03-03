from nav_algo.sensors import sensorData
import nav_algo.nmea as nmea
import nav_algo.coordinates as coord
from camera import Camera


class sensorArray(sensorData):
    def __init__(self):
        # IMU
        self.pitch = 0
        self.roll = 0
        # we read as wrt N, convert to wrt x-axis (E) (make sure 90 degrees is north)
        self.yaw = 0

        # anemometer
        self.wind_direction = 0  # wrt x-axis
        self.wind_speed = 0

        # GPS
        self.fix = False
        self.latitude = 0
        self.longitude = 0
        self.velocity = coord.Vector()

        # camera
        self.camera = Camera
        self.buoyCoords = []
        self.boatCoords = (0, 0)

    def updateAll(self):
        self.readGPS()
        self.readIMU()
        self.readWindDirection()
        self.readCamera()

    def readIMU(self):
        # TODO remember to convert yaw to wrt x-axis
        pass

    def readWindDirection(self):
        # TODO remember we read this as wrt yaw, to get wrt x-axis, add yaw
        # and use coord.rangeAngle to get into the range [0, 360)
        pass

    def readGPS(self):
        # TODO update fix, lat, long, and velocity
        # use the NMEA parser
        pass

    def readCamera(self):
        # TODO update buoy list based on which buoys not already in list of obstacles
        pass
