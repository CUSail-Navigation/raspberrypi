import nav_algo.servo as servo
import nav_algo.sensors as sens
import nav_algo.coordinates as coord


class BoatController:

    def __init__(self, 
                 coordinate_system=None, 
                 sensor_data=None, 
                 mock_servos=False):

        # Set the sensor object if given or make a new one
        if sensor_data is None:
            self.sensors = sens.sensorData(coordinate_system)
        else:
            self.sensors = sensor_data

        # Set the servo object with indicator to fake the servos or not
        self.servos = servo.Servo(mock_servos)

    def getPosition(self):
        return self.sensors.position

    def updateSensors(self):
        self.sensors.readAll()

    def setServos(self, mainsail: float, tail: float):
        self.servos.setSail(mainsail)
        self.servos.setTail(tail)
