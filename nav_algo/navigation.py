import time
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
import nav_algo.sim_gui as gui
from nav_algo.events import Events
from nav_algo.navigation_helper import *


class NavigationController:
    """A controller class for the navigation algorithm.

    Args:
        waypoints (list of (float, float)): A list of (latitude, longitude) tuples of waypoints.
        simulation (bool): (Optional) A parameter used to bypass blocking calls.

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

    def __init__(self, event=None, waypoints=[], simulation=False):
        self.DETECTION_RADIUS = 5.0

        if not simulation:
            self.coordinate_system = coord.CoordinateSystem(
                waypoints[0][0], waypoints[0][1])
            self.waypoints = [
                coord.Vector(self.coordinate_system, w[0], w[1])
                for w in waypoints
            ]

            self.boat = boat.BoatController(
                coordinate_system=self.coordinate_system)

            self.radio = radio.Radio()
            self.radio.transmitString("Waiting for GPS fix...\n")

            # wait until we know where we are
            while not self.boat.sensors.fix:
                self.boat.sensors.readGPS()  # ok if this is blocking

            self.radio.transmitString(
                "Established GPS fix. Beginning navigation...\n")
            self.current_waypoint = self.waypoints.pop(0)

            if event == Events.ENDURANCE:
                self.endurance()
            elif event == Events.STATION_KEEPING:
                # to find an optimal radius, 10 for now
                exit_before = 300
                circle_radius = 10
                self.stationKeeping(self.waypoints, circle_radius, "ENTRY")
                self.navigate()
                # Set timer
                start_time = time.time()
                loop_waypoints = self.stationKeeping(
                    self.waypoints, circle_radius, "KEEP")
                self.navigate()
                while time.time() - start_time < exit_before:
                    self.waypoints = loop_waypoints
                    self.navigate()
                self.stationKeeping(self.waypoints, circle_radius, "EXIT")
            elif event == Events.PRECISION_NAVIGATION:
                self.precisionNavigation()
            elif event == Events.COLLISION_AVOIDANCE:
                self.collisionAvoidance()
            elif event == Events.SEARCH:
                self.search()

            self.navigate()

        else:
            self.boat = boat.BoatController(simulation=True)
            self.gui = gui.GUI(self.boat)

    def navigate(self):
        """ Execute the navigation algorithm.

        This is a blocking call that runs until all waypoints have been hit.

        """
        while self.current_waypoint is not None:
            time.sleep(2)  # TODO how often should this run?

            self.boat.updateSensors()
            self.boat_position = self.boat.getPosition()

            if self.boat_position.xyDist(
                    self.current_waypoint) < self.DETECTION_RADIUS:
                if len(self.waypoints) > 0:
                    self.current_waypoint = self.waypoints.pop(0)
                else:
                    self.current_waypoint = None
                    break

            sailing_angle = newSailingAngle(self.boat, self.current_waypoint)
            self.boat.setServos(sailing_angle)

        # TODO cleanup pins?

    def endurance(self):
        # TODO do setup and then call nav helper endurance function
        pass

    def stationKeeping(self, waypoints, circle_radius, state):
        stationKeeping(waypoints, circle_radius, state)

    def precisionNavigation(self,waypoints):
        # TODO do setup and then call nav helper precision navigation function
        precisionNavigation(waypoints)

    def collisionAvoidance(self):
        # TODO do setup and then call nav helper collision avoidance function
        pass

    def search(self):
        # TODO do setup and then call nav helper search function
        pass
