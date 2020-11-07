from RPIO import PWM
from SailSensors import I2CDevice

class servo(I2CDevice):

    TAIL_MAX_ANGLE = 148
    TAIL_MIN_ANGLE = 0
    SAIL_MAX = 1924.65
    SAIL_MIN = 696.15

    TAIL_MAX_ANGLE = 60
    TAIL_MIN_ANGLE = 0
    TAIL_MAX = 1368.73
    TAIL_MIN = 704.34

    """
    instantiates the class. tailPin is the otherwise unused GPIO pin on the pi where the
    tail servo is connected, and sailPin is the otherwise unused GPIO pin where the sail
    servo is connected.
    """
    def __init__(self, i2caddress = 0xE0, i2cBus = 0, tailPin, sailPin):
        super().__init__(i2caddress, i2cBus)
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
        offset = bytes([8 + 4 *  self.tailPin])
        intOnPer = mapRange(tail_angle,TAIL_MIN_ANGLE,TAIL_MAX_ANGLE,TAIL_MIN,TAIL_MAX)
        timePeriodByte = bytes([intOnPer])
        self.writeBlockData(timePeriodByte,offset[0])
        return

    """
    Just instantiate the class then enter an angle into "servo"_angle as an
    int SAIL_MIN_ANGLE -> SAIL_MAX_ANGLE degrees
    """
    def setSail(self, sail_angle):
        offset = bytes([8 + 4 *  self.sailPin])
        intOnPer = round(mapRange(sail_angle,SAIl_MIN_ANGLE,SAIL_MAX_ANGLE,SAIL_MIN,SAIL_MAX))
        timePeriodByte = bytes([intOnPer])
        self.writeBlockData(timePeriodByte,offset[0])
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
    def mapRange(val,min,max,endMin,endMax):
        return ((val/(max - min))*(endMax-endMin)) + endMin
