


from adafruit_servokit import ServoKit
import time



class Servo:
    SAIL_MAX_ANGLE = 90
    SAIL_MIN_ANGLE = -90

    TAIL_MAX_ANGLE = 30
    TAIL_MIN_ANGLE = -30

    def __init__(self, mock=False):
        """
        instantiates the class. tailPin is the otherwise unused GPIO pin on the pi where the
        tail servo is connected, and sailPin is the otherwise unused GPIO pin where the sail
        servo is connected.
        """
        self.mock = mock
        if self.mock:
            self.currentSail = 0
            self.currentTail = 0
            return
        
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
        if tail_angle < Servo.TAIL_MIN_ANGLE:
            tail_angle = Servo.TAIL_MIN_ANGLE
        elif tail_angle > Servo.TAIL_MAX_ANGLE:
            tail_angle = Servo.TAIL_MAX_ANGLE
            
        intOnPer = 50 - tail_angle
        self.servoDriver.servo[1].angle = intOnPer
    
    def setSail(self, sail_angle):
        """
        Just instantiate the class then enter an angle into "servo"_angle as an
        int SAIL_MIN_ANGLE -> SAIL_MAX_ANGLE degrees
        """
        if sail_angle < Servo.SAIL_MIN_ANGLE:
            sail_angle = Servo.SAIL_MIN_ANGLE
        elif sail_angle > Servo.SAIL_MAX_ANGLE:
            sail_angle = Servo.SAIL_MAX_ANGLE

        self.currentSail = sail_angle
        if self.mock:
            return
    
        intOnPer = self.mapRange(sail_angle,-90,90,30,135)
        self.servoDriver.servo[0].angle = intOnPer
        
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
        raise NotImplementedError("Encoder? I hardly know her")

    def mapRange(self, val, startmin, startmax, endMin, endMax):
        """
        Returns value based on a linear map of MIN -> MAX to a value ENDMIN -> ENDMAX
        """
        return (((val - startmin) / (startmax - startmin)) * (endMax - endMin)) + endMin


servoController = Servo()
print("done")

servoController.setSail(0)
servoController.setTail(0)

