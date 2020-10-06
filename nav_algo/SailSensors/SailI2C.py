"""
This class is used to create a new instance of an I2C sensor module for communication.
It also contains functions to communicate with the module.
"""
from smbus2 import SMBus, i2c_msg

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
    """
    def __init__(self,deviceAddress,i2cBusIndex):
        self.deviceAddress = deviceAddress
        self.i2cBusIndex = i2cBusIndex
        self.i2cBus = SMBus(self.i2cBusIndex)
        return

    """

    """
    def readBlockData(self,offset,byteNumber):
        return self.i2cBus.read_i2c_block_data(self.deviceAddress,offset,byteNumber)



    """
    Sends a hexadecimal list to the desired component.
    list msg (needs to be a list of hexadecimal commands specific to the device)
    """
    def i2cWr(self,msg):
        send = i2c_msg.write(self.getDeviceAddress(),msg)
        self.geti2cBus().i2c_rdwr(send)
        return

    """
    Sends a hexadecimal-command list to the desired component. The function then
    waits for and retrieves a response from the device with a length based on
    int recieve. Returns a list of bytes with length of int recieve
    list send (sends this list of hexadecimal commands)
    int recieve(the amount of bytes in a list to retrieve from the device)

    """
    def i2cRdwr(self,send,recieve):
        write = i2c_msg.write(self.getDeviceAddress(),send)
        recieve = i2c_msg.read(self.getDeviceAddress(),recieve)
        return self.geti2cBus().i2c_rdwr(write,recieve)
