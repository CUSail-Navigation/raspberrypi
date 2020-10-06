"""
SailIMU implements a class used to connect the component to it's proper
communication protocol. This class also implements functions to return raw data and
turn on/off the sensors.
"""

from smbus2 import SMBus, i2c_msg
from SailI2C import I2CDevice

class SailIMU(I2CDevice):
    imuCommands = {
    "readAccelerometerRaw": 0x42,
    "readOrientationEuler": 0x01,
    "readCompassRaw": 0x43
    }

    def __init__(self,deviceAddress,i2cBusIndex):
        super().__init__(deviceAddress,i2cBusIndex)
        return

    def i2c_read_imu(self):
        msg = [SailIMU.imuCommands["readAccelerometerRaw"],SailIMU.imuCommands["readOrientationEuler"]]
        self.i2cWr(msg)
        msg = [SailIMU.imuCommands["readCompassRaw"]]
        self.i2cWr(msg)
        data = self.readBlockData(0x0,12)
        return data
