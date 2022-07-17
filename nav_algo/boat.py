import nav_algo.servo as servo
import nav_algo.sensors as sens
import nav_algo.coordinates as coord
import nav_algo.navigation_utilities as util
import numpy as np


class BoatController:

    def __init__(self, coordinate_system=None):
        self.coordinate_system = coordinate_system
        self.sensors = sens.sensorData(coordinate_system)

        # servo angles
        self.servos = servo.Servo()
        self.sail_angle = 0
        self.tail_angle = 0

    def getPosition(self):
        return self.sensors.position

    def updateSensors(self):
        self.sensors.readAll()

    def getServoAngles(self, intended_angle: float):
        # TODO check logic for all of this, I'm 99% sure it's wrong - CM
        abs_wind_dir = self.sensors.wind_direction
        yaw = self.sensors.yaw
        return util.getServoAnglesImpl(abs_wind_dir, yaw, intended_angle)

    def setServos(self, intended_angle: float):
        self.sail_angle, self.tail_angle = self.getServoAngles(intended_angle)

        # set the servos
        print("setting sail {} tail {}".format(self.sail_angle,
                                               self.tail_angle))
        self.servos.setTail(self.tail_angle)
        self.servos.setSail(self.sail_angle)
        self.sensors.sailAngleBoat = self.servos.currentSail

    def setAngles(self, mainsail: float, tail: float):
        print("setting sail {} tail {}".format(mainsail, tail))
        self.servos.setSail(mainsail)
        self.servos.setTail(tail)