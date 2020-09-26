"""
SailAnemometer implements a class used to connect the component to it's proper
communication protocol. This class also implements functions to return raw data and
turn on/off the sensors.
"""

import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


class SailAnemometer:

    def __init__(self):
        return
