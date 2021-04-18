from adafruit_servokit import ServoKit


class Servo:
    SAIL_MAX_ANGLE = 148
    SAIL_MIN_ANGLE = 0
    SAIL_MAX = 180
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
        self.currentTail = 0
        self.currentSail = 0
        self.servoDriver = ServoKit(channels=16)
        return

    def setTail(self, tail_angle):
        """
        Just instantiate the class then enter an angle into "servo"_angle as an
        int TAIL_MIN_ANGLE -> TAIL_MAX_ANGLE degrees
        """
        if tail_angle <= Servo.TAIL_MAX_ANGLE and tail_angle >= Servo.TAIL_MIN_ANGLE:
            intOnPer = self.mapRange(tail_angle, Servo.TAIL_MIN_ANGLE,
                                     Servo.TAIL_MAX_ANGLE, Servo.TAIL_MIN,
                                     Servo.TAIL_MAX)
            self.servoDriver.servo[0].angle = intOnPer
            self.currentTail = tail_angle
        else:
            print("Did not set tail, desired angle not between " +
                  str(Servo.TAIL_MIN_ANGLE) + "and" +
                  str(Servo.TAIL_MAX_ANGLE))
        return

    def setSail(self, sail_angle):
        """
        Just instantiate the class then enter an angle into "servo"_angle as an
        int SAIL_MIN_ANGLE -> SAIL_MAX_ANGLE degrees
        """
        if sail_angle <= Servo.SAIL_MAX_ANGLE and sail_angle >= Servo.SAIL_MIN_ANGLE:
            intOnPer = self.mapRange(sail_angle, Servo.SAIL_MIN_ANGLE,
                                     Servo.SAIL_MAX_ANGLE, Servo.SAIL_MIN,
                                     Servo.SAIL_MAX)
            self.servoDriver.servo[1].angle = intOnPer
            self.currentTail = sail_angle
        else:
            print("Did not set sail, desired angle not between " +
                  str(Servo.SAIL_MIN_ANGLE) + "and" +
                  str(Servo.SAIL_MAX_ANGLE))
        return

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

    def mapRange(self, val, min, max, endMin, endMax):
        """
        Returns value based on a linear map of MIN -> MAX to a value ENDMIN -> ENDMAX
        """
        return ((val / (max - min)) * (endMax - endMin)) + endMin
