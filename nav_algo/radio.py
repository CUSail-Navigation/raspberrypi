from SailSensors import UARTDevice

# TODO document this class
class Radio(UARTDevice):
    def __init__(self, baudrate,serialPort = '/dev/ttyAMA0',t = 1):
        super().__init__(baudrate,serialPort,t)
        pass

    """
    Prints the given messge to the basestation.

    """
    def transmitString(self, message: str):
        self.sendUart(message)
        pass

    def printData(self):
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
        # TODO write a better docstring
        pass

    def printAllWaypoints(self):
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
        # TODO write a better docstring
        pass

    def printHitWaypoint(self):
        """Data should be of the form:.

        "----------HIT----------" +
        ",X:" + waypoint.x + " Y:" + waypoint.y +
        ",----------END----------" + new line character

        Note that fields are comma delineated while x and y coordinates of
        the same point are space delineated.

        Note 'printAllWaypoints' should be called immediately after this.

        """
        # TODO write a better docstring
        pass
