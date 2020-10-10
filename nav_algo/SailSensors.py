
from smbus2 import SMBus, i2c_msg
from Adafruit_ADS1x15 import ADS1x15 as ADS

IMU_ADDRESS = 0x77
ADC_ADDRESS = 0x48

"""
This class is used to create a new instance of an I2C sensor module for communication.
It also contains functions to communicate with the module.
"""
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
    Reads a block of data from the i2c device register.
    Parameters:
    -byte offset: integer value for if you want to offset your data collection from the register
    by a ceratain amount of bytes(default 0)
    -int bytenumber the number of bytes you would like to read from the device's register
    """
    def readBlockData(self,byteNumber,offset = 0x0):
        return self.i2cBus.read_i2c_block_data(self.deviceAddress,offset,byteNumber)



    """
    Sends a hexadecimal list to the desired component.
    parameters:
    -byte list msg (needs to be a list of hexadecimal commands specific to the device)
    """
    def i2cWr(self,msg):
        send = i2c_msg.write(self.getDeviceAddress(),msg)
        self.geti2cBus().i2c_rdwr(send)
        return

    """
    Sends a hexadecimal-command list to the desired component. The function then
    waits for and retrieves a response from the device with a length based on
    parameters:
    -int recieve. Returns a list of bytes with length of int recieve
    -byte list send (sends this list of hexadecimal commands)
    -int recieve(the amount of bytes in a list to retrieve from the device)

    """
    def i2cRdwr(self,send,recieve):
        write = i2c_msg.write(self.getDeviceAddress(),send)
        recieve = i2c_msg.read(self.getDeviceAddress(),recieve)
        return self.geti2cBus().i2c_rdwr(write,recieve)
    
"""
This class contains the object of the ADC to be used by other
classes for analog devices. 
"""
class ADCDevice:
    
    mainADC = ADS.ADS1015(ADC_ADDRESS)
    
    """
    Init function for an arbitrary ADCDevice
    Parameter:
    -pinNumber: must be an int between 0 and 3, based which pin on the
    ADC the sensor is connected to.
    """
    def __init__(self,pinNumber):
        self.pinNumber = pinNumber
        return
        
    
    """This function reads the ADC present on the I2C bus
        Parameters:
        -gain: The gain of the sensor input in int form
    """
    def readADC(self,gain = 1):
        return ADCDevice.mainADC.read_adc(self.pinNumber)
        
"""
SailIMU implements a class used to connect the component to it's proper
communication protocol. This class also implements functions to return raw data and
turn on/off the sensors.
"""
class SailIMU(I2CDevice):
    imuCommands = {
    "readAccelerometerRaw": 0x42,
    "readOrientationEuler": 0x01,
    "readCompassRaw": 0x43
    }
    """
    Initializes the sensor object, default for i2cBusIndex is 1 because the raspberry pi only has 1 bus
    """
    def __init__(self,deviceAddress = IMU_ADDRESS,i2cBusIndex = 1):
        super().__init__(deviceAddress,i2cBusIndex)
        return

    """
    Reads the IMU and returns a list of 12 bytes representing euler angles.
    """
    def i2c_read_imu(self):
        msg = [SailIMU.imuCommands["readAccelerometerRaw"],SailIMU.imuCommands["readOrientationEuler"]]
        self.i2cWr(msg)
        msg = [SailIMU.imuCommands["readCompassRaw"]]
        self.i2cWr(msg)
        data = self.readBlockData(12)
        return data



"""
SailEncoder implements a class used to connect the component to it's proper
communication protocol. This class also implements functions to return raw data and
turn on/off the sensors.
"""
class SailEncoder:

    def __init__(self):
        return
    
    

"""
SailGPS implements a class used to connect the component to it's proper
communication protocol. This class also implements functions to return raw data and
turn on/off the sensors.
"""

class SailGPS:

    def __init__(self):
        return


"""
SailAnemometer implements a class used to connect the component to it's proper
communication protocol. This class also implements functions to return raw data and
turn on/off the sensors.
"""
class SailAnemometer(ADCDevice):
    
    """
    Initializes the anemometer.
    Parameter:
    -pinNumber: pin the anemometer is plugged into in the form of an
    int between 0 and 3
    """
    def __init__(self,pinNumber):
        super().__init__(pinNumber)
        
    """
    Returns the given voltage of the anemometer using the ADC.
    Parameter:
    -gain: the multiplication value of the voltage, must be an int
    """
    def readAnemometerVoltage(self,gain = 1):
        return self.readADC(gain)
