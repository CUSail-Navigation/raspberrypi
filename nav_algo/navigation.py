import time

import nav_algo.configuration as conf
import nav_algo.basic_algo as algo
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
from nav_algo.events import Events
from nav_algo.camera import Camera
from nav_algo.event_helper.station_keeping_event.station_keeping import stationKeeping
from nav_algo.event_helper.endurance_event.endurance import endurance
from nav_algo.event_helper.fleetrace_event.fleetrace import fleetRace
from nav_algo.event_helper.precision_nav_event.precision_nav import precisionNavigation
from nav_algo.event_helper.search_event.search import search
from nav_algo.event_helper.collision_event.collision_avoidance import collision_avoidance


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
        
        # New tacking fields
        self.tacking = False
        self.tackingPoint = None
        self.tackingDuration = 0
        self.distToDest = 0

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
            fleetRace(self)

        elif self.configuration.event == Events.ENDURANCE:
            endurance(self)

        elif self.configuration.event == Events.STATION_KEEPING:
            stationKeeping(self)

        elif self.configuration.event == Events.PRECISION_NAVIGATION:
            self.DETECTION_RADIUS = 4  # Be more precise
            self.configuration.waypoints = precisionNavigation(self)
            self.current_waypoint = self.configuration.waypoints.pop(0)
            self.navigate(self)

        elif self.configuration.event == Events.COLLISION_AVOIDANCE:
            collision_avoidance(self)

        elif self.configuration.event == Events.SEARCH:
            search(self)

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
                print('fails')
                pass

            # Check if manual override has been engaged
            if self.configuration.radio is not None and self.configuration.radio.fleetRace:
                fleetRace(self)

            # Print all waypoints (in order from first to last)
            all_waypts = []
            all_waypts.append(self.current_waypoint)
            for pt in self.configuration.waypoints:
                all_waypts.append(pt)
            self.configuration.write_waypoints(all_waypts)

            # Sleep for a small amount of time to let the boat move
            time.sleep(1)  # This should never be more than 1 second

            # Get the updated sensor readings and print them
            self.configuration.boat.updateSensors()
            boat_position = self.configuration.boat.getPosition()
            self.configuration.write_data(self.current_waypoint)

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
            # Old sensor/algo command:
            # sail, rudder = self.configuration.algo.step(
            #     self.configuration.boat, self.current_waypoint)
            sail, rudder, tacking, tpoint, tduration = self.configuration.algo.step(boat_position, self.current_waypoint, self.tacking, self.tackingPoint, self.tackingDuration, self.configuration.boat.sensors.yaw, self.configuration.boat.sensors.wind_direction)
            self.configuration.boat.setServos(sail, rudder)
            self.tacking, self.tackingPoint, self.tackingDuration = tacking, tpoint, tduration
