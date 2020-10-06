import SailSensors.SailI2C
"""
Various tests to see if the IMU communicates properly.
"""

def test():
    testIMU = sailIMU(0x77,1);
    print("IMU Reads: " +testIMU.i2c_read_lidar())
