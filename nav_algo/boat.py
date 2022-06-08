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
        
        sail = 0
        angle_of_attack = self.sensors.wind_direction - self.sensors.yaw
        if abs(angle_of_attack) > 180.0:
            angle_of_attack = (abs(angle_of_attack) - 180) * -1 + 180
            
        if abs(angle_of_attack) > 160.0:
            sail = 90.0 * np.sign(angle_of_attack)
        elif abs(angle_of_attack) > 110.0:
            sail = 60.0 * np.sign(angle_of_attack)
        elif abs(angle_of_attack) > 75.0:
            sail = 45.0 * np.sign(angle_of_attack)
        elif abs(angle_of_attack) > 35.0:
            sail = 30.0 * np.sign(angle_of_attack)
        else:
            sail = 15.0 * np.sign(angle_of_attack)

        offset = intended_angle - self.sensors.yaw 
        # map to range of servos
        if(abs(angle_of_attack) < 15):
            tail = -30
        else:
            tail = round(offset)  #+ 30.0
        #sail = sail + 74.0

        if (tail > 30):
            tail = 30
        if (tail < -30):
            tail = -30

        return sail, tail

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