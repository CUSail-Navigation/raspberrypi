from RPIO import PWM

class servo:

    TAIL_MAX_ANGLE = 148
    TAIL_MIN_ANGLE = 0
    SAIL_MAX = 0
    SAIL_MIN = 0

    TAIL_MAX_ANGLE = 60
    TAIL_MIN_ANGLE = 0
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
    Just instantiate the class then enter an angle into "servo"_angle as an
    int TAIL_MIN_ANGLE -> TAIL_MAX_ANGLE degrees
    """
    def setTail(self, tail_angle):
        mapAngle = (tail_angle / (servo.TAIL_MAX_ANGLE - servo.TAIL_MIN_ANGLE)) * (servo.TAIL_MAX - servo.TAIL_MIN) + servo.TAIL_MIN
        self.servoControl.set_servo(self.tailPin,mapAngle)
        self.currentTail = tail_angle
        return

    """
    Just instantiate the class then enter an angle into "servo"_angle as an
    int SAIL_MIN_ANGLE -> SAIL_MAX_ANGLE degrees
    """
    def setSail(self, sail_angle):
        mapAngle = (sail_angle / (servo.SAIL_MAX_ANGLE - servo.SAIL_MIN_ANGLE)) * (servo.SAIL_MAX - servo.SAIL_MIN) + servo.SAIL_MIN
        self.servoControl.set_servo(self.sailPin,mapAngle)
        self.currentSail = sail_angle
        return

    def readEncoder(self):
        print("Encoder?! I hardly know her!")
        return
