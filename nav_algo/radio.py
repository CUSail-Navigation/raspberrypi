from nav_algo.low_level.SailSensors import UARTDevice
import nav_algo.boat as boat
import nav_algo.coordinates as coord
from time import time


class Radio(UARTDevice):
    """
    Constructor for the radio.
    -baudrate is an integer(should be 9600 for the xbee)
    -serialport, the serial port the xbee is connected to(just leave default for the xbee)
    -t is the uart timeout period(just leave at 1 for normal operation)
    """
    def __init__(self,
                 baudrate,
                 boatController=None,
                 fleetRace=False,
                 serialPort='/dev/ttyAMA1',
                 t=1):
        super().__init__(baudrate, serialPort, t, fleetRace)
        self.boatController = boatController

    """
    Prints the given messge to the basestation. string must be sent with a 'b' before the string
    """

    def transmitString(self, message: str):
        print(message)
        self.sendUart(message.encode('utf-8'))
        pass

    """
    Reads in a line from the XBee. NOTE this assumes that the line ends with \n
    If 'q' is received, the nav algo will quit.
    """

    def receiveString(self):
        l = self.readline()
        l = l.replace('\n', '')
        if l == 'q':
            print("Quitting...")
            self.sendUart("Quitting...".encode('utf-8'))
            time.sleep(1)  # give time to send message, then quit
            self.serialStream.close()
            self.boatController.sensors.gps_serial_port.close()
            raise RuntimeError('Quitting navigation algorithm.')
        elif l == 'o':
            # manual override
            print("Entering Manual Override...")
            self.sendUart("Entering Manual Override...".encode('utf-8'))
            self.fleetRace = True
        elif l == 'a':
            # turn on autopilot
            print("Entering Autopilot Mode...")
            self.sendUart("Entering Autopilot Mode...".encode('utf-8'))
            self.fleetRace = False
        elif self.fleetRace:
            # assumes the only other possibility is setting sail angles
            self.readAngles(l)

    """
    Reads in manual sail and rudder (tail sail) angles of the form "a b" (space delineated)
    """

    def readAngles(self, message: str):
        spl = message.split(" ")
        if not len(spl) == 2:
            self.sendUart(
                "Angles in incorrect format. Ignoring.".encode('utf-8'))
            return

        sail = float(spl[0])
        tail = float(spl[1])
        self.boatController.setAngles(sail, tail)
