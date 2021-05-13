import nav_algo.servo as servo
import nav_algo.sensors as sens
import nav_algo.coordinates as coord


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
        # based on previous algorithm and Wikipedia, 15 degrees is critical angle of attack
        angle_of_attack = 15.0
        if self.sensors.wind_direction < 180.0:
            angle_of_attack = -15.0

        offset = self.sensors.yaw - intended_angle
        # -90 to put in sero coords
        tail = round(self.sensors.wind_direction + offset) - 90
        sail = round(tail + angle_of_attack) - 90

        # convert sail and tail from wrt north to wrt boat
        tail = tail - self.sensors.yaw
        sail = sail - self.sensors.yaw

        # put in range [0, 360)
        tail = coord.rangeAngle(tail)
        sail = coord.rangeAngle(sail)

        return sail, tail

    def setServos(self, intended_angle: float):
        self.sail_angle, self.tail_angle = self.getServoAngles(intended_angle)

        # set the servos
        self.servos.setTail(self.tail_angle)
        self.servos.setSail(self.sail_angle)