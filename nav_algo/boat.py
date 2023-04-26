import nav_algo.servo as servo
import nav_algo.sensors as sens
import nav_algo.coordinates as coord
import nav_algo.navigation_utilities as util
import numpy as np


class BoatController:

    def __init__(self, 
                 coordinate_system=None, 
                 sensor_data=None, 
                 mock_servos=False):
        self.coordinate_system = coordinate_system
        self.mock_servos = mock_servos

        if sensor_data is None:
            self.sensors = sens.sensorData(coordinate_system)
        else:
            self.sensors = sensor_data

        # servo angles
        self.servos = servo.Servo(mock_servos)

    def getPosition(self):
        return self.sensors.position

    def updateSensors(self):
        self.sensors.readAll()

    def setServos(self, mainsail: float, tail: float):
        print("setting sail {} tail {}".format(mainsail, tail))
        self.servos.setSail(mainsail)
        self.servos.setTail(tail)