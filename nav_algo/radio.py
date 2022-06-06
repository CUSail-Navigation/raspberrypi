from nav_algo.SailSensors import UARTDevice
import nav_algo.boat as boat
import nav_algo.coordinates as coord
from time import time
import sys


# TODO document this class
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
                 fleetRace = False,
                 serialPort='/dev/ttyS0',
                 t=1):
        super().__init__(baudrate, serialPort, t , fleetRace)
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
            sys.exit()
            time.sleep(1)  # give time to send message, then quit 
        else:
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

    """
    Sends all of the boat data to the basestation. All arguments are taken in as floats
    """

    def printData(self, boatController):
        """Data should be of the form:.

        "----------NAVIGATION----------" +
        ",Origin Latitude: " + origLat +
        ",Origin Longitude: " + origLong +
        ",X position: " + currentPosition.x +
        ",Y position: " + currentPosition.y +
        ",Wind Direction: " + windDir +
        ",Pitch: " + pitch +
        ",Roll: " + roll +
        ",Yaw: " + yaw +
        ",Sail Angle: " + sailAngle +
        ",Tail Angle: " + tailAngle +
        ",Heading: " + heading +
        ",----------END----------" + new line character

        Note that fields are comma delineated and there is only a new line
        character at the end of the string.

        """
        origLat = boatController.coordinate_system.LAT_OFFSET
        origLong = boatController.coordinate_system.LONG_OFFSET
        currentPositionX = boatController.sensors.position.x
        currentPositionY = boatController.sensors.position.y
        windDir = boatController.sensors.wind_direction
        pitch = boatController.sensors.pitch
        roll = boatController.sensors.roll
        yaw = boatController.sensors.yaw
        sailAngle = boatController.sail_angle
        tailAngle = boatController.tail_angle
        heading = boatController.sensors.velocity.angle()

        msg = ("----------NAVIGATION----------" + ",Origin Latitude: " +
               str(origLat) + ",Origin Longitude: " + str(origLong) +
               ",X position: " + str(currentPositionX) + ",Y position: " +
               str(currentPositionY) + ",Wind Direction: " + str(windDir) +
               ",Pitch: " + str(pitch) + ",Roll: " + str(roll) + ",Yaw: " +
               str(yaw) + ",Sail Angle: " + str(sailAngle) + ",Tail Angle: " +
               str(tailAngle) + ",Heading: " + str(heading) +
               ",----------END----------" + '\n')
        print(msg)
        msg = msg.encode()
        self.sendUart(msg)
        return
        """
        Takes a list of tuple waypoints and sends them to the basestation.
        -currentWaypointsArray: must be an array of tuples in the format (waypoint x component float, waypoint y component float)
        """

    def printAllWaypoints(self, currentWaypointsArray):
        """Data should be of the form:.

        "----------WAYPOINTS----------" +
        ",X:" + current_waypoint.x + " Y:" + current_waypoint.y +
        ",X:" + next_waypoint.x + " Y:" + next_waypoint.y +
        ...
        ",X:" + last_waypoint.x + " Y:" + last_waypoint.y +
        ",----------END----------" + new line character

        Note that waypoints are comma delineated while x and y coordinates of
        the same point are space delineated. The waypoints should be printed in
        order from first to last (do not include waypoints that have already
        been hit).
        """
        msg = "----------WAYPOINTS----------"
        for j in currentWaypointsArray:
            msg = msg + ",X:" + str(j.x) + " Y:" + str(j.y)
        pass
        msg = msg + ",----------END----------" + '\n'
        print(msg)
        msg = msg.encode()
        self.sendUart(msg)
        return

    """
    Sends a single waypoint(meant to be the waypoint the boat just hit) to the basestation.
    -hitWaypoint: a tuple in the format (waypoint x component float, waypoint y component float)
    """

    def printHitWaypoint(self, hitWaypoint):
        """Data should be of the form:.

        "----------HIT----------" +
        ",X:" + waypoint.x + " Y:" + waypoint.y +
        ",----------END----------" + new line character

        Note that fields are comma delineated while x and y coordinates of
        the same point are space delineated.

        Note 'printAllWaypoints' should be called immediately after this.

        """
        msg = ("----------HIT----------" + ",X:" + hitWaypoint[0] + " Y:" +
               hitWaypoint[1] + ",----------END----------" + '\n')
        print(msg)
        msg = msg.encode()
        self.sendUart(msg)
        return
