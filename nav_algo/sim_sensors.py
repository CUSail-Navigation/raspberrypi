from nav_algo.sensors import sensorData
import nav_algo.coordinates as coord


class sensorSim(sensorData):
    def __init__(self):
        # IMU
        self.pitch = 0
        self.roll = 0
        self.yaw = 0  # we read as wrt N, convert to wrt x-axis (E) (make sure 90 degrees is north)

        # anemometer
        self.wind_direction = 5  # wrt x-axis
        self.wind_speed = 2

        # GPS
        self.position = coord.Vector(x=0, y=0)
        self.velocity = coord.Vector(x=1, y=1)

    def readAll(self):
        pass
