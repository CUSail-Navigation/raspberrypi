from smbus2 import SMBus, i2c_msg
from Adafruit_ADS1x15 import ADS1x15 as ADS
import serial

IMU_ADDRESS = 0x77
ADC_ADDRESS = 0x48


class I2CDevice:
    """This class is used to create a new instance of an I2C sensor module for communication.
    It also contains functions to communicate with the module.
    """
    def getDeviceAddress(self):
        return self.deviceAddress

    def geti2cBusIndex(self):
        return self.i2cBusIndex

    def geti2cBus(self):
        return self.i2cBus

    def getName(self):
        return self.name

    def __init__(self, deviceAddress, i2cBusIndex):
        """initializes the class.

        Args:
            deviceAddress (int): needs to be found using sudo i2cdetect -y 1.
            i2cBusIndex (int): Should be 0, b/c there is only one bus on the pi.
        """
        self.deviceAddress = deviceAddress
        self.i2cBusIndex = i2cBusIndex
        self.i2cBus = SMBus(self.i2cBusIndex)
        return

    def readBlockData(self, byteNumber, offset=0x0):
        """Reads a block of data from the i2c device register.

        Args:
            bytenumber (int): the number of bytes you would like to read from the device's register.
            offset (int): integer value for the byte offset of data collection from the register.
        """
        return self.i2cBus.read_i2c_block_data(self.deviceAddress, offset,
                                               byteNumber)

    def writeBlockData(self, msg, offset=0x0):
        """Writes a block of data from the i2c device register.

        Args:
            msg (list): list of bytes to send.
            offset (int): integer value for the byte offset of data collection from the register.
        """
        return self.i2cBus.write_i2c_block_data(self.deviceAddress, offset,
                                                msg)

    def i2cWr(self, msg):
        """Sends a hexadecimal list to the desired component.

        Args:
            msg (list): list of bytes (needs to be a list of hexadecimal commands specific to the device).
        """
        send = i2c_msg.write(self.getDeviceAddress(), msg)
        self.geti2cBus().i2c_rdwr(send)
        return

    def i2cRdwr(self, send, recieve):
        """Sends a hexadecimal-command list to the desired component.
        The function then waits for and retrieves a response from the device 
        with a length based on the parameters.

        Args:
            send (list): a list of bytes (sends this list of hexadecimal commands).
            recieve (int): the amount of bytes in a list to retrieve from the device.
        
        Returns:
            list: a list of bytes with length of int recieve.
        """
        write = i2c_msg.write(self.getDeviceAddress(), send)
        recieve = i2c_msg.read(self.getDeviceAddress(), recieve)
        return self.geti2cBus().i2c_rdwr(write, recieve)


class UARTDevice:
    """
    Superclass used to create a UARTDevice. Contians various functions for communication
    """
    def __init__(self, baudrate, serialPort='/dev/ttyS0', t=1):
        self.port = serialPort
        # open the port only when necessary
        self.serialStream = serial.Serial(port=None,
                                          baudrate=baudrate,
                                          timeout=t)
        self.serialStream.port = self.port
        return

    def sendUart(self, message):
        self.serialStream.open()
        self.serialStream.write(message)
        self.serialStream.flush()
        self.serialStream.close()
        return

    def recieveUartBytes(self, bytes=1):
        # TODO this may need to be open constantly in order to read
        self.serialStream.open()
        self.serialStream.read(bytes)  #TODO you'll also want to return this
        self.serialStream.close()
        return

    def readline(self):
        # TODO port may need to be open constantly to read...
        self.serialStream.open()
        l = self.serialStream.readline()
        self.serialStream.close()
        return l


class ADCDevice:
    """
    This class contains the object of the ADC to be used by other
    classes for analog devices.
    """
    mainADC = ADS.ADS1015(ADC_ADDRESS)

    def __init__(self, pinNumber):
        """Init function for an arbitrary ADCDevice.
        
        Args:
            pinNumber (int): between 0 and 3, which pin on the ADC the sensor is connected to.
        """
        self.pinNumber = pinNumber
        return

    def readADC(self, gain=1):
        """This function reads the ADC present on the I2C bus.

        Args:
            gain (int): The gain of the sensor input.
        """
        return ADCDevice.mainADC.read_adc(self.pinNumber, gain)


class SailIMU(I2CDevice):
    """
    SailIMU implements a class used to connect the component to it's proper
    communication protocol. This class also implements functions to return raw data and
    turn on/off the sensors.
    """
    imuCommands = {
        "readAccelerometerRaw": 0x42,
        "readOrientationEuler": 0x01,
        "readCompassRaw": 0x43
    }

    def __init__(self, deviceAddress=IMU_ADDRESS, i2cBusIndex=1):
        """
        Initializes the sensor object, default for i2cBusIndex is 1 because the raspberry pi only has 1 bus
        """
        super().__init__(deviceAddress, i2cBusIndex)
        return

    def i2c_read_imu(self):
        """
        Reads the IMU and returns a list of 12 bytes representing euler angles.
        """
        msg = [
            SailIMU.imuCommands["readAccelerometerRaw"],
            SailIMU.imuCommands["readOrientationEuler"]
        ]
        self.i2cWr(msg)
        msg = [SailIMU.imuCommands["readCompassRaw"]]
        self.i2cWr(msg)
        data = self.readBlockData(12)
        return data


class SailEncoder:
    """
    SailEncoder implements a class used to connect the component to it's proper
    communication protocol. This class also implements functions to return raw data and
    turn on/off the sensors.
    """
    def __init__(self):
        return


class SailGPS:
    """
    SailGPS implements a class used to connect the component to it's proper
    communication protocol. This class also implements functions to return raw data and
    turn on/off the sensors.
    """
    def __init__(self):
        return


class SailAnemometer(ADCDevice):
    """
    SailAnemometer implements a class used to connect the component to it's proper
    communication protocol. This class also implements functions to return raw data and
    turn on/off the sensors.
    """
    def __init__(self, pinNumber):
        """Initializes the anemometer.
        
        Args:
            pinNumber (int): between 0 and 3, pin the anemometer is plugged into.
        """
        super().__init__(pinNumber)

    def readAnemometerVoltage(self, gain=2 / 3):
        """Returns the given voltage of the anemometer using the ADC.
        
        Args:
            gain (int): the multiplication value of the voltage
        """
        return self.readADC(gain)
