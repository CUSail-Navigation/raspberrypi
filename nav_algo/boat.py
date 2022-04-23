import nav_algo.servo as servo
import nav_algo.sensors as sens
import nav_algo.coordinates as coord
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
        angle_of_attack = 15.0
        if self.sensors.wind_direction < 180.0:
            angle_of_attack = -15.0
        sail = 0
        angle_of_attack = intended_angle - self.sensors.wind_direction
        if abs(angle_of_attack) < 15.0:
            sail = 90.0 * np.sign(angle_of_attack)
        elif abs(angle_of_attack) < 45.0:
            sail = 20.0 * np.sign(angle_of_attack)
        elif abs(angle_of_attack) < 75.0:
            sail = 45.0 * np.sign(angle_of_attack)
        elif abs(angle_of_attack) < 105.0:
            sail = 60.0 * np.sign(angle_of_attack)
        elif abs(angle_of_attack) < 135.0:
            sail = 75.0 * np.sign(angle_of_attack)
        else:
            sail = 90.0 * np.sign(angle_of_attack)

        offset = self.sensors.yaw - intended_angle
        # map to range of servos
        tail = round(offset) #+ 30.0
        #sail = sail + 74.0

        # put in range [0, 360)
        tail = coord.rangeAngle(tail)
        sail = coord.rangeAngle(sail)

        return sail, tail

    def setServos(self, intended_angle: float):
        self.sail_angle, self.tail_angle = self.getServoAngles(intended_angle)

        # set the servos
        print("setting sail {} tail {}".format(self.sail_angle, self.tail_angle))
        self.servos.setTail(self.tail_angle)
        self.servos.setSail(self.sail_angle)