import time

import nav_algo.configuration as conf
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
from nav_algo.events import Events
from nav_algo.navigation_helper import *
from nav_algo.camera import Camera


class NavigationController:
    """A controller class for the navigation algorithm.
    Args:
        configuration (NavigationConfiguration: The configuration of the current execution.
    Attributes:
        DETECTION_RADIUS (float): How close we need to get to a waypoint.
        coordinate_system (CoordinateSystem): The global coordinate system.
        waypoints (list of Vector): Position vectors of waypoints.
        boat (BoatController): A representation of the boat.
        radio (Radio): Prints navigation data to the base station.
        current_waypoint (Vector): The current target waypoint.
        boat_position (Vector): The current position of the boat.
        boat_to_target (Vector): The vector from the boat to the target position.
    """
    def __init__(self, configuration: conf.NavigationConfiguration):
        self.configuration = configuration
        self.DETECTION_RADIUS = 5.0

        self.configuration.write_output(
            "Using lat/long point ({}, {}) as the center of the coordinate system.\n"
            .format(self.configuration.waypoints[0].latitude,
                    self.configuration.waypoints[0].longitude))
        self.configuration.write_output("Waiting for GPS fix...\n")

        # wait until we know where we are
        while self.configuration.boat.sensors.velocity is None:
            self.configuration.boat.sensors.readAll()  # ok if this is blocking

        self.configuration.write_output(
            "Established GPS fix. Beginning navigation...\n")

        # If the event is fleet race, we don't care about the algo, just set angles
        # NOTE commands should end with \n, send 'q' to quit, angles are space delineated 'main tail'
        if self.configuration.event == Events.FLEET_RACE:
            self.fleetRace()

        elif self.configuration.event == Events.ENDURANCE:
            self.endurance()

        elif self.configuration.event == Events.STATION_KEEPING:
            self.stationKeeping()

        elif self.configuration.event == Events.PRECISION_NAVIGATION:
            self.DETECTION_RADIUS = 1.5  # Be more precise
            self.configuration.waypoints = precisionNavigation(
                self.configuration.waypoints)
            self.current_waypoint = self.configuration.waypoints.pop(0)
            self.navigate()

        elif self.configuration.event == Events.COLLISION_AVOIDANCE:
            self.collision_avoidance()

        elif self.configuration.event == Events.SEARCH:
            self.search()

        else:
            # No event provided, just follow waypoints directly
            self.current_waypoint = self.configuration.waypoints.pop(0)
            self.navigate()

        # Clean up ports
        self.configuration.write_output("HIT ALL WAYPOINTS. EXITING.")
        self.configuration.cleanup()

    def navigate(self, use_camera=False):
        """ Execute the navigation algorithm.
        This is a blocking call that runs until all waypoints have been hit.
        If the camera is used, the call will terminate when anything is detected
        and return the coordinates of the thing detected.
        """
        while self.current_waypoint is not None:
            # read for a quit signal ('q') or manual override ('o')
            try:
                if self.configuration.radio is not None:
                    self.configuration.radio.receiveString()
            except:
                pass

            # Check if manual override has been engaged
            if self.configuration.radio is not None and self.configuration.radio.fleetRace:
                self.fleetRace()

            # Print all waypoints (in order from first to last)
            all_waypts = []
            all_waypts.append(self.current_waypoint)
            for pt in self.configuration.waypoints:
                all_waypts.append(pt)
            self.configuration.write_waypoints(all_waypts)

            # Sleep for a small amount of time to let the boat move
            time.sleep(0.25)  # TODO how often should this run?

            # Get the updated sensor readings and print them
            self.configuration.boat.updateSensors()
            boat_position = self.configuration.boat.getPosition()
            self.configuration.write_data()

            # Check if we've reached the current waypoint and get the next one
            if boat_position.xyDist(
                    self.current_waypoint) < self.DETECTION_RADIUS:
                # hit waypoint -- send data back to basestation
                self.configuration.write_hit_waypoint(self.current_waypoint)

                if len(self.configuration.waypoints) > 0:
                    self.current_waypoint = self.configuration.waypoints.pop(0)
                else:
                    self.current_waypoint = None
                    break

            # If we're using the camera, get an image and check for buoy or boat
            if use_camera:
                yaw = self.configuration.boat.sensors.yaw
                x = boat_position.x
                y = boat_position.y

                # Check for buoy if we're doing search
                if self.configuration.event == Events.SEARCH:
                    buoy_loc = self.camera.read_buoy(yaw, x, y)
                    if buoy_loc is not None:
                        return buoy_loc

                # Check for boat if we're doing collision avoidance
                if self.configuration.event == Events.COLLISION_AVOIDANCE:
                    boat_loc = self.camera.read_boat(yaw, x, y)
                    if boat_loc is not None:
                        return boat_loc

            # Run the algorithm to get the desired sail and rudder angles
            sail, rudder = self.configuration.algo.step(
                self.configuration.boat, self.current_waypoint)
            self.configuration.boat.setServos(sail, rudder)

    def fleetRace(self):
        # While the configuration is in fleet race mode, read servo angles
        # over the radio
        self.configuration.write_output(
            "Starting Fleet Race\nSend angles of the form 'sail_angle rudder_angle'\n"
        )
        while self.configuration.radio.fleetRace:
            try:
                self.configuration.radio.receiveString()  # timeout is 1 sec
            except:
                pass
            self.configuration.boat.updateSensors()
            self.configuration.write_data()

    def endurance(self):
        # Loop around the same waypoints for 7 hours.
        # 7 hrs = 25200 sec
        exit_before = 25200
        start_time = time.time()
        loop_waypoints = counterClockwiseRect(self.configuration.waypoints,
                                              self.configuration.boat,
                                              buoy_offset=5)

        while (time.time() - start_time < exit_before):
            self.configuration.waypoints = loop_waypoints
            self.current_waypoint = self.configuration.waypoints.pop(0)
            self.navigate()

    def stationKeeping(self):
        # TODO find an optimal radius, 10m for now
        buoy_waypoints = self.configuration.waypoints
        exit_before = 300
        circle_radius = 10
        self.configuration.waypoints = stationKeeping(
            buoy_waypoints,
            circle_radius,
            "ENTRY",
            boat=self.configuration.boat)
        self.current_waypoint = self.configuration.waypoints.pop(0)
        self.navigate()

        # Set timer
        start_time = time.time()
        loop_waypoints = stationKeeping(buoy_waypoints,
                                        circle_radius,
                                        "KEEP",
                                        boat=self.configuration.boat)
        while time.time() - start_time < exit_before:
            self.configuration.waypoints = loop_waypoints
            self.current_waypoint = self.configuration.waypoints.pop(0)
            self.navigate()

        self.configuration.waypoints = stationKeeping(
            buoy_waypoints,
            circle_radius,
            "EXIT",
            boat=self.configuration.boat)

    def search(self):
        self.camera = Camera()
        search_radius = 80
        num_seeds = 15

        # Assuming the input waypoint (origin) is the middle of the search field
        # Seed a bunch of random waypoints throughtout the environment
        buoy_loc = None
        while buoy_loc is None:
            waypoints = []
            for _ in range(num_seeds):
                angle = 2 * np.pi * np.random.rand()
                radius = np.random.rand() * search_radius
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                w = coord.Vector(x=x, y=y)
                waypoints.append(w)
            self.configuration.waypoints = waypoints
            self.current_waypoint = self.configuration.waypoints.pop(0)

            # Navigate between the seed waypoints until we see the buoy
            buoy_loc = self.navigate(use_camera=True)

        # Go to the buoy location
        coord_sys = self.configuration.boat.sensors.coordinate_system
        buoy_loc = coord.Vector.convertXYToLatLong(coord_sys, buoy_loc.x,
                                                   buoy_loc.y)
        self.current_waypoint = buoy_loc
        self.configuration.waypoints = []
        self.DETECTION_RADIUS = 1.0
        self.navigate(use_camera=False)

        # Signal that we've found the buoy
        # TODO do something more interesting
        self.configuration.write_output("FOUND_BUOY")
        self.configuration.write_output("BUOY LAT: {} LONG: {}".format(
            buoy_loc.latitude, buoy_loc.longitude))

        # Station Keeping Mode
        while True:
            self.current_waypoint = buoy_loc
            self.navigate(use_camera=False)

    def collision_avoidance(self):
        self.camera = Camera()

        # Get waypoints between the input buoys
        self.configuration.waypoints = collisionAvoidance(
            self.configuration.waypoints)
        self.current_waypoint = self.configuration.waypoints.pop(0)

        # Navigate until we see a boat
        boat_loc = self.navigate(use_camera=True)
        while boat_loc is not None:
            # See the boat, push the rudder to one side to spin away for a while
            # TODO what should we really do here?
            self.configuration.write_output("SEE BOAT AT ({}, {})".format(
                boat_loc.x, boat_loc.y))
            self.configuration.boat.setServos(60.0, 20.0)
            time.sleep(10.0)  # Give time to move away
            boat_loc = self.navigate(use_camera=True)
