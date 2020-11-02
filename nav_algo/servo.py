from RPIO import PWM

class servo:

    SAIL_MAX = 0
    SAIL_MIN = 0
    TAIL_MAX = 1670
    TAIL_MIN = 858

    """
    instantiates the class. tailPin is the otherwise unused GPIO pin on the pi where the
    tail servo is connected, and sailPin is the otherwise unused GPIO pin where the sail
    servo is connected. 
    """
    def __init__(self,tailPin,sailPin):
        self.currentTail = 0
        self.currentSail = 0
        self.tailPin = tailPin
        self.sailPin = sailPin
        self.servoControl = PWM.Servo()
        return

    """
    Just instantiate the class then enter an angle into "servo"_angle as an int 0->180 degrees
    """
    def setTail(self, tail_angle):
        mapAngle = (tail_angle / 180) * (TAIL_MAX - TAIL_MIN) + 858
        self.servoControl.set_servo(tailPin,mapAngle)
        self.currentTail = tail_angle
        return

    """
    Just instantiate the class then enter an angle into "servo"_angle as an int 0->180 degrees
    """
    def setSail(self, sail_angle):
        mapAngle = (sail_angle / 180) * (SAIL_MAX - SAIL_MIN) + 858
        self.servoControl.set_servo(sailPin,mapAngle)
        self.currentSail = sail_angle
        return

    def readEncoder(self):
        print("Encoder?! I hardly know her!")
        return
