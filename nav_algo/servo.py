from adafruit_servokit import ServoKit


class Servo:
    SAIL_MAX_ANGLE = 90
    SAIL_MIN_ANGLE = 90
    SAIL_MAX = 0
    SAIL_MIN = 0

    TAIL_MAX_ANGLE = 60
    TAIL_MIN_ANGLE = 0
    TAIL_MAX = 180
    TAIL_MIN = 0

    def __init__(self):
        """
        instantiates the class. tailPin is the otherwise unused GPIO pin on the pi where the
        tail servo is connected, and sailPin is the otherwise unused GPIO pin where the sail
        servo is connected.
        """
        self.servoDriver = ServoKit(channels=16)
        self.currentTail = 0
        self.setTail(0)
        self.currentSail = 0
        self.setSail(0)
        return

    def setTail(self, tail_angle):
        """
        Just instantiate the class then enter an angle into "servo"_angle as an
        int TAIL_MIN_ANGLE -> TAIL_MAX_ANGLE degrees
        """
        if tail_angle > Servo.TAIL_MAX_ANGLE:
            tail_angle = Servo.TAIL_MAX_ANGLE
        elif tail_angle < Servo.TAIL_MIN_ANGLE:
            tail_angle = Servo.TAIL_MIN_ANGLE
    
        intOnPer = self.mapRange(tail_angle, Servo.TAIL_MIN_ANGLE, Servo.TAIL_MAX_ANGLE, Servo.TAIL_MIN, Servo.TAIL_MAX)
        self.servoDriver.servo[1].angle = intOnPer
        self.currentTail = tail_angle

    def setSail(self, sail_angle):
        """
        Just instantiate the class then enter an angle into "servo"_angle as an
        int SAIL_MIN_ANGLE -> SAIL_MAX_ANGLE degrees
        """
        if sail_angle < Servo.SAIL_MIN_ANGLE:
            sail_angle = Servo.SAIL_MIN_ANGLE
        elif sail_angle > Servo.SAIL_MAX_ANGLE:
            sail_angle = Servo.SAIL_MAX_ANGLE
    
        intOnPer = 3 * (sail_angle + 180) * (180/1800)
        self.servoDriver.servo[0].angle = intOnPer
        self.currentSail = sail_angle

    def sleepServo(self, sleep):
        """Puts the servo driver to sleep to conserve energy.

        Args:
            sleep (bool): turns off oscillator if True, if false it runs in normal operation.
        """
        if sleep == True:
            self.writeBlockData(0x11)  #Sets the sleep register to 1
        else:
            self.writeBLockData(0x01)  #Sets the sleep register to 0
        return

    def readEncoder(self):
        print("encoder?")
        return

    def mapRange(self, val, startmin, startmax, endMin, endMax):
        """
        Returns value based on a linear map of MIN -> MAX to a value ENDMIN -> ENDMAX
        """
        return ((val / (startmax - startmin)) * (endMax - endMin)) + endMin
