import json
import nav_algo.sensors as sens
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
from nav_algo.events import Events
from nav_algo.rl_algo import RL
from nav_algo.basic_algo import BasicAlgo

class NavigationConfiguration:

    def __init__(self, config_filename, waypoint_filename):
        obj = None
        with open(config_filename) as f:
            obj = json.load(f)
        
        # Figure out if sensors are real or fake (mocked)
        mock_gps = obj["peripherals"]["gps"] == "fake"
        mock_imu = obj["peripherals"]["imu"] == "fake"
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
                                      mock_gps=mock_gps,
                                      mock_imu=mock_imu,
                                      mock_anemometer=mock_anemometer)
        
        # Figure out if the servos are real or mocked and setup boat object
        mock_servos = obj["peripherals"]["servos"] == "fake"
        self.boat = boat.BoatController(coord_sys, sensor_data, mock_servos)

        # Determine whether the reinforcement learning or basic algorithm is used
        self.algo = None
        if obj["algo"]["type"] == "rl":
            self.algo = RL(obj["algo"]["model_path"])
        else:
            self.algo = BasicAlgo() # TODO
        
        # Figure out which event is being run
        self.event = None
        if obj["algo"]["event"] == "endurance":
            self.event = Events.ENDURANCE
        elif obj["algo"]["event"] == "station keeping":
            self.event = Events.STATION_KEEPING
        elif obj["algo"]["event"] == "precision navigation":
            self.event = Events.PRECISION_NAVIGATION
        elif obj["algo"]["event"] == "collision avoidance":
            self.event = Events.COLLISION_AVOIDANCE
        elif obj["algo"]["event"] == "search":
            self.event = Events.SEARCH
        elif obj["algo"]["event"] == "fleet race":
            obj["output"] = "radio" # must get input from radio
            self.event = Events.FLEET_RACE

        # Figure out if the radio is being used or just print statements
        if obj["output"] == "radio":
            self.radio = radio.Radio(9600, 
                                     boatController=self.boat, 
                                     fleetRace=(self.event == Events.FLEET_RACE))
    
    def write_output(self, message):
        if self.radio is None:
            print(message, end='')
        else:
            self.radio.transmitString(message)

    def write_data(self):
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
        origLat = self.boat.coordinate_system.LAT_OFFSET
        origLong = self.boat.coordinate_system.LONG_OFFSET
        currentPositionX = self.boat.sensors.position.x
        currentPositionY = self.boat.sensors.position.y
        windDir = self.boat.sensors.wind_direction
        pitch = self.boat.sensors.pitch
        roll = self.boat.sensors.roll
        yaw = self.boat.sensors.yaw
        sailAngle = self.boat.servos.currentSail
        tailAngle = self.boat.servos.currentTail
        heading = self.boat.sensors.velocity.angle()

        msg = ("----------NAVIGATION----------" + ",Origin Latitude: " +
               str(origLat) + ",Origin Longitude: " + str(origLong) +
               ",X position: " + str(currentPositionX) + ",Y position: " +
               str(currentPositionY) + ",Wind Direction: " + str(windDir) +
               ",Pitch: " + str(pitch) + ",Roll: " + str(roll) + ",Yaw: " +
               str(yaw) + ",Sail Angle: " + str(sailAngle) + ",Tail Angle: " +
               str(tailAngle) + ",Heading: " + str(heading) +
               ",----------END----------" + '\n')
        
        self.write_output(msg)

    def readWaypoints(self, filename):
        waypoints = []
        with open(filename, 'r') as f:
            for line in f:
                line = line.replace('\n',"")
                
                l = line.split(",")
                waypoints.append((float(l[0]), float(l[1])))
        return waypoints

    def cleanup(self):
        # TODO should anything else go here?
        if self.radio is not None:
            self.radio.serialStream.close()

