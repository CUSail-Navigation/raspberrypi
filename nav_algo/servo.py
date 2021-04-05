from adafruit_servokit import ServoKit


class servo:

    SAIL_MAX_ANGLE = 148
    SAIL_MIN_ANGLE = 0
    SAIL_MAX = 180
    SAIL_MIN = 0

    TAIL_MAX_ANGLE = 60
    TAIL_MIN_ANGLE = 0
    TAIL_MAX = 180
    TAIL_MIN = 0

    """
    instantiates the class. tailPin is the otherwise unused GPIO pin on the pi where the
    tail servo is connected, and sailPin is the otherwise unused GPIO pin where the sail
    servo is connected.
    """
    def __init__(self, tailPin, sailPin):
        self.currentTail = 0
        self.currentSail = 0
        self.tailPin = tailPin
        self.sailPin = sailPin
        self.servoDriver = ServoKit(channels = 16)
        return

    """
    Just instantiate the class then enter an angle into "servo"_angle as an
    int TAIL_MIN_ANGLE -> TAIL_MAX_ANGLE degrees
    """
    def setTail(self, tail_angle):
        if tail_angle <= servo.TAIL_MAX_ANGLE and tail_angle >= servo.TAIL_MIN_ANGLE:
            intOnPer = self.mapRange(tail_angle,servo.TAIL_MIN_ANGLE,servo.TAIL_MAX_ANGLE,servo.TAIL_MIN,servo.TAIL_MAX)
            self.servoDriver.servo[0].angle = intOnPer
            self.currentTail = tail_angle
        else:
            print("Did not set tail, desired angle not between " + str(servo.TAIL_MIN_ANGLE) +"and" +str(servo.TAIL_MAX_ANGLE))
        return

    """
    Just instantiate the class then enter an angle into "servo"_angle as an
    int SAIL_MIN_ANGLE -> SAIL_MAX_ANGLE degrees
    """
    def setSail(self, sail_angle):
        if sail_angle <= servo.SAIL_MAX_ANGLE and sail_angle >= servo.SAIL_MIN_ANGLE:
            intOnPer = self.mapRange(sail_angle,servo.SAIL_MIN_ANGLE,servo.SAIL_MAX_ANGLE,servo.SAIL_MIN,servo.SAIL_MAX)
            self.servoDriver.servo[1].angle = intOnPer
            self.currentTail = sail_angle
        else:
            print("Did not set sail, desired angle not between " + str(servo.SAIL_MIN_ANGLE) +"and" +str(servo.SAIL_MAX_ANGLE))
        return

    """
    Puts the servo driver to sleep to conserve energy.
    -boolean sleep: turns off oscillator if True, if false it runs in normal operation.
    """
    def sleepServo(self,sleep):
        if sleep == True:
            self.writeBlockData(0x11) #Sets the sleep register to 1
        else:
            self.writeBLockData(0x01) #Sets the sleep register to 0
        return

    def readEncoder(self):
        print("encoder?")
        return

    """
    Returns value based on a linear map of MIN -> MAX to a value ENDMIN -> ENDMAX
    """
    def mapRange(self,val,min,max,endMin,endMax):
        return ((val/(max - min))*(endMax-endMin)) + endMin
