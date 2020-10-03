"""
SailIMU implements a class used to connect the component to it's proper
communication protocol. This class also implements functions to return raw data and
turn on/off the sensors.
"""

from smbus2 import SMBus, ic_msg

class SailIMU(I2CDevice):

    def __init__(self,deviceAddress,i2cBusIndex,name):
        super().__init__(deviceAddress,i2cBusIndex,name)
        return

    def i2c_read_lidar(self):
        msg = [66,1,67]
        data = self.i2cRdwr(msg)
        return data
