import time

import nav_algo.configuration as conf
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
from nav_algo.events import Events
from nav_algo.navigation_helper import *
# from nav_algo.camera import Camera


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
    def __init__(self, configuration : conf.NavigationConfiguration):
        self.configuration = configuration
        self.DETECTION_RADIUS = 5.0

        self.configuration.write_output(
            "Using lat/long point ({}, {}) as the center of the coordinate system.\n"
            .format(self.configuration.waypoints[0].latitude, self.configuration.waypoints[0].longitude))
        self.configuration.write_output("Waiting for GPS fix...\n")

        # wait until we know where we are
        while self.configuration.boat.sensors.velocity is None:
            self.configuration.boat.sensors.readAll() # ok if this is blocking
        
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
            self.configuration.waypoints = precisionNavigation(self.configuration.waypoints)
            self.current_waypoint = self.configuration.waypoints.pop(0)
            self.navigate()

        # TODO actually implement collision avoidance
        elif self.configuration.event == Events.COLLISION_AVOIDANCE:
            self.configuration.waypoints = collisionAvoidance(self.configuration.waypoints)
            self.current_waypoint = self.configuration.waypoints[0]
            self.navigateDetection()

        # TODO actually implement search
        elif self.configuration.event == Events.SEARCH:
            self.configuration.waypoints = search(self.configuration.waypoints, 
                                                  boat=self.configuration.boat)
            self.current_waypoint = self.configuration.waypoints[0]
            self.navigateDetection(event=Events.SEARCH)

        else:
            # No event provided, just follow waypoints directly
            self.current_waypoint = self.configuration.waypoints.pop(0)
            self.navigate()

        # Clean up ports
        self.configuration.cleanup()

    def navigate(self):
        """ Execute the navigation algorithm.
        This is a blocking call that runs until all waypoints have been hit.
        """
        while self.current_waypoint is not None:
            # read for a quit signal ('q') or manual override ('o')
            try:
                if self.configuration.radio is not None:
                    self.configuration.radio.receiveString()
            except:
                pass

            # Check if manual override has been engaged
            if self.radio is not None and self.configuration.radio.fleetRace:
                self.fleetRace()

            # Print all waypoints (in order from first to last)
            all_waypts = []
            all_waypts.append(self.current_waypoint)
            for pt in self.configuration.waypoints:
                all_waypts.append(pt)
            self.configuration.write_waypoints(all_waypts)

            # Sleep for a small amount of time to let the boat move
            time.sleep(1)  # TODO how often should this run?
            
            # Get the updated sensor readings and print them
            self.configuration.boat.updateSensors()
            boat_position = self.configuration.boat.getPosition()
            self.configuration.write_data()

            # Check if we've reached the current waypoint and get the next one
            if boat_position.xyDist(self.current_waypoint) < self.DETECTION_RADIUS:
                # hit waypoint -- send data back to basestation
                self.configuration.write_hit_waypoint(self.current_waypoint)

                if len(self.configuration.waypoints) > 0:
                    self.current_waypoint = self.configuration.waypoints.pop(0)
                else:
                    self.current_waypoint = None
                    break

            # Run the algorithm to get the desired sail and rudder angles
            sail, rudder = self.configuration.algo.step()
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
        self.configuration.waypoints = stationKeeping(buoy_waypoints,
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

        self.configuration.waypoints = stationKeeping(buoy_waypoints,
                                                      circle_radius,
                                                      "EXIT",
                                                      boat=self.configuration.boat)
    
    def navigateDetection(self, event=Events.COLLISION_AVOIDANCE):
        # TODO does this need to be different than navigate()?
        # vision needs a complete refactor either way
        raise NotImplementedError("CV needs a complete refactor")
