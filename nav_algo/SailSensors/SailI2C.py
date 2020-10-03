"""
This class is used to create a new instance of an I2C sensor module for communication.
It also contains functions to communicate with the module.
"""
from smbus2 import SMBus, ic_msg

class I2CDevice:

    def getDeviceAddress(self):
        return self.deviceAddress

    def geti2cBusIndex(self):
        return self.i2cBusIndex

    def geti2cBus(self):
        return self.i2cBus

    def getName(self):
        return self.name

    """
    initializes the class
    int deviceAddress (needs to be found using sudo i2cdetect -y 1)
    int i2cBusIndex (can be an arbitrary int between 0-7)
    string name (name of the device, like 'IMU' or whatever you want)
    """
    def __init__(self,deviceAddress,i2cBusIndex,name):
        self.deviceAddress = deviceAddress
        self.i2cBusIndex = i2cBusIndex
        self.i2cBus = SMBus(self.i2cBusIndex)
        self.name = name
        return

    """

    """
    def read_block_data(self,offset,byteNumber):
        return self.i2cBus.read_i2c_block_data(deviceAdddress,offset,byteNumber)

    def i2cRdwr(self,msg):
        send = i2c_msg.write(self.getDeviceAddress(),msg)
        returnToSender = self.geti2cBus().i2c_rdwr(send)
        return returnToSender
