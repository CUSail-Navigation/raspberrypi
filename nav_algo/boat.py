import servo as servo
import sensors as sens
import coordinates as coord


class BoatController:
    def __init__(self):
        self.sensors = sens.sensorData()

        # servo angles
        # TODO print these to the gui
        self.sail_angle = 0
        self.tail_angle = 0

    def getPosition(self, coordinate_system):
        return coord.Vector(coordinate_system, self.sensors.latitude,
                            self.sensors.longitude)

    def updateSensors(self, simulation=False):
        pass

    def setServos(self, intended_angle: float):
        # TODO check logic for all of this, I'm 99% sure it's wrong - CM
        # based on previous algorithm and Wikipedia, 15 degrees is critical angle of attack
        angle_of_attack = 15.0
        if self.sensors.wind_direction < 180.0:
            angle_of_attack = -15.0

        offset = self.sensors.yaw - intended_angle
        tail = round(wind_angle + offset)
        sail = round(tail + angle_of_attack)

        # put in range [0, 360)
        tail = ((tail % 360) + 360) % 360
        sail = ((sail % 360) + 360) % 360

        # convert sail and tail from wrt north to wrt boat
        tail = tail - self.sensors.yaw
        sail = sail - self.sensors.yaw

        # set the servos
        self.tail_angle = servo.setTail(coord.rangeAngle(tail))
        self.sail_angle = servo.setSail(coord.rangeAngle(sail))