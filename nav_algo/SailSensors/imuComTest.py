from SailIMU import SailIMU
"""
Various tests to see if the IMU communicates properly.
"""

def test():
    testIMU = SailIMU(0x77,1);
    print(testIMU.i2c_read_imu());
    return

test()


