import json
import nav_algo.sensors as sens
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
from nav_algo.events import Events
from nav_algo.basic_algo import BasicAlgo


class NavigationConfiguration:
    """ This object represents the configuration of the current instance of the
    navigation algorithm. It tracks the waypoints as well as the boat controller
    and the algorithm being used, etc.

    It also provides a wrapper interface for printing output. It provides a 
    high level function that other classes can call to write output. This 
    object will then decide whether to output to the terminal or to the radio.
    """
    def __init__(self, config_filename, waypoint_filename, event=None):
        obj = None
        with open(config_filename) as f:
            obj = json.load(f)

        # Figure out if sensors are real or fake (mocked)
        mock_airmar = obj["peripherals"]["airmar"] == "fake"
        mock_anemometer = obj["peripherals"]["anemometer"] == "fake"

        # Get the coordinate system with the origin at the first waypoint
        waypoints = self.readWaypoints(waypoint_filename)
        if len(waypoints) < 1:
            raise RuntimeError('At least one waypoint is required.')

        coord_sys = coord.CoordinateSystem(waypoints[0][0], waypoints[0][1])
        self.waypoints = [
            coord.Vector(coord_sys, w[0], w[1]) for w in waypoints
        ]

        # Get the sensor data object with mocked or real sensors
        sensor_data = sens.sensorData(coordinate_system=coord_sys,
                                      mock_airmar=mock_airmar,
                                      mock_anemometer=mock_anemometer)

        # Figure out if the servos are real or mocked and setup boat object
        mock_servos = obj["peripherals"]["servos"] == "fake"
        self.boat = boat.BoatController(coord_sys, sensor_data, mock_servos)

        self.algo = BasicAlgo()

        # Figure out which event is being run
        self.event = event
        if self.event == Events.FLEET_RACE:
            obj["output"] = "radio"  # must get input from radio

        # Figure out if the radio is being used or just print statements
        if obj["output"] == "radio":
            self.radio = radio.Radio(
                9600,
                boatController=self.boat,
                fleetRace=(self.event == Events.FLEET_RACE))
        else:
                self.radio = None

    def write_output(self, message):
        if self.radio is None:
            print(message, end='')
        else:
            print("radio msg: " + message)
            self.radio.transmitString(message)

    def write_data(self, current_waypoint=None):
        """Data should be of the form:.

        "----------NAVIGATION----------" +
        ",Origin Latitude: " + origLat +
        ",Origin Longitude: " + origLong +
        ",X position: " + currentPosition.x +
        ",Y position: " + currentPosition.y +
        ",Wind Direction: " + windDir +
        ",Relative wind: " + relative_wind +
        ",Pitch: " + pitch +
        ",Roll: " + roll +
        ",Yaw: " + yaw +
        ",Sail Angle: " + sailAngle +
        ",Tail Angle: " + tailAngle +
        ",Heading: " + heading +
        ",Angular velocity: " + angular_velocity +
        ",X velocity: " + velocity.x +
        ",Y velocity: " + velocity.y + 
        ",X waypoint: " + current_waypoint.x + 
        ",Y waypoint: " + current_waypoint.y +
        ",----------END----------" + new line character

        Note that fields are comma delineated and there is only a new line
        character at the end of the string.
        """

        origLat = self.boat.sensors.coordinate_system.LAT_OFFSET
        origLong = self.boat.sensors.coordinate_system.LONG_OFFSET
        currentPositionX = self.boat.sensors.position.x
        currentPositionY = self.boat.sensors.position.y
        windDir = self.boat.sensors.wind_direction
        relWind = self.boat.sensors.relative_wind
        pitch = self.boat.sensors.pitch
        roll = self.boat.sensors.roll
        yaw = self.boat.sensors.yaw
        sailAngle = self.boat.servos.currentSail
        tailAngle = self.boat.servos.currentTail
        heading = self.boat.sensors.velocity.angle()
        ang_vel = self.boat.sensors.angular_velocity
        velocity = self.boat.sensors.velocity
        waypointX = current_waypoint.x if current_waypoint is not None else None
        waypointY = current_waypoint.y if current_waypoint is not None else None

        msg = ("----------NAVIGATION----------" + ",Origin Latitude: " +
               str(origLat) + ",Origin Longitude: " + str(origLong) +
               ",X position: " + str(currentPositionX) + ",Y position: " +
               str(currentPositionY) + ",Wind Direction: " + str(windDir) +
               ",Relative wind: " + str(relWind) + ",Pitch: " + str(pitch) +
               ",Roll: " + str(roll) + ",Yaw: " + str(yaw) + ",Sail Angle: " +
               str(sailAngle) + ",Tail Angle: " + str(tailAngle) +
               ",Heading: " + str(heading) + ",Angular velocity: " +
               str(ang_vel) + ",X velocity: " + str(velocity.x) +
               ",Y velocity: " + str(velocity.y) + ",X waypoint: " +
               str(waypointX) + ",Y waypoint: " + str(waypointY) +
               ",----------END----------" + '\n')

        self.write_output(msg)

    def write_waypoints(self, currentWaypointsArray):
        """Data should be of the form: list of vectors.

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
        msg = msg + ",----------END----------" + '\n'
        self.write_output(msg)

    def write_hit_waypoint(self, hitWaypoint):
        """Data should be of the form: vector.

        "----------HIT----------" +
        ",X:" + waypoint.x + " Y:" + waypoint.y +
        ",----------END----------" + new line character

        Note that fields are comma delineated while x and y coordinates of
        the same point are space delineated.

        Note 'printAllWaypoints' should be called immediately after this.

        """
        msg = ("----------HIT----------" + ",X:" + str(hitWaypoint.x) + " Y:" +
               str(hitWaypoint.y) + ",----------END----------" + '\n')
        self.write_output(msg)

    def readWaypoints(self, filename):
        waypoints = []
        with open(filename, 'r') as f:
            for line in f:
                line = line.replace('\n', "")

                l = line.split(",")
                waypoints.append((float(l[0]), float(l[1])))
        return waypoints

    def cleanup(self):
        # TODO should anything else go here?
        if self.radio is not None:
            self.radio.serialStream.close()
