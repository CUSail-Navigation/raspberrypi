
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
    int i2cBusIndex (Should be 0, b/c there is only one bus on the pi)
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
    Writes a block of data from the i2c device register.
    Parameters:
    -byte offset: integer value for if you want to offset your data collection from the register
    by a ceratain amount of bytes(default 0)
    -list of bytes msg
    """
    def writeBlockData(self, msg, offset = 0x0):

        return self.i2cBus.write_i2c_block_data(self.deviceAddress,offset,msg)


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

    """
    Reads data sent to GPS and translates into coordinates.
    """
class readGPS (self):
    lat = [];
        longi = [];

        latIdx = 0;
        longIdx = 0;

        east = -1;
        north = 1;

        valid = 0;

        #only parse GPGLL sentences (for now)
        if (data[0] == '$' and data[1] == 'G' and data[2] == 'P' and data[3] == 'G'
            and data[4] == 'L' and data[5] == 'L'):
            fieldNum = 0; # keep track of how many fields have been parsed
            i = 6;
            done = 0;

            while (i < 82 and done == 0):
                    i += 1;
                    if (data[i] == ','):
                        fieldNum += 1;

                    elif (fieldNum == 0):
                        lat[latIdx] = data[i];
                        latIdx += 1;

                    elif (fieldNum == 1):
                        if (data[i] == 'N'):
                            north = 1
                        else:
                            north = -1

                    elif (fieldNum == 2):
                        longi[longIdx] = data[i];
                        longIdx += 1;

                    elif (fieldNum == 3):
                        if (data[i] == 'E'):
                            east = 1
                        else:
                            east = -1

                    elif (fieldNum == 5 and data[i] == 'A'):
                        valid = 1;


        elif (data[0] == '$' and data[1] == 'G' and (data[2] == 'P' or data[2] == 'N')
                and data[3] == 'G'and data[4] == 'G' and data[5] == 'A' and data[19] != '*'):
                fieldNum = 0; # keep track of how many fields have been parsed
                i = 6;
                done = 0;

        while (i < 82 and done == 0):
            i += 1
            if (data[i] == ','):
                fieldNum += 1;
            elif (fieldNum == 1):
                lat[latIdx] = data[i];
                latIdx += 1;
            elif (fieldNum == 2):
                if (data[i] == 'N'):
                    north = 1;
                else:
                    north = -1;
            elif (fieldNum == 3):
                longi[longIdx] = data[i];
                longIdx += 1;
            elif (fieldNum == 4):
                if (data[i] == 'E'):
                    east = 1
                else:
                    east = -1;
            elif (fieldNum == 5 and data[i] != '0'):
                valid = 1;




        if (valid):
            lat_deg[2], longi_deg[3], lat_min[10], longi_min[10];

            lat_deg[0] = lat[0];
            lat_deg[1] = lat[1];
            longi_deg[0] = longi[0];
            longi_deg[1] = longi[1];
            longi_deg[2] = longi[2];
            idx = 0;
            while (idx < 10):
                lat_min[idx] = lat[idx+2];
                longi_min[idx] = longi[idx+3];
                idx += 1;

            lat_dd, longi_dd, lat_mm, longi_mm;
            lat_dd = atof(lat_deg);
            longi_dd = atof(longi_deg);
            lat_mm = atof(lat_min);
            longi_mm = atof(longi_min);

            (sensorData).lat = north * (lat_dd + lat_mm/60.0);
            (sensorData).longi = east * (longi_dd + longi_mm/60.0);
            convertLLtoXY();
