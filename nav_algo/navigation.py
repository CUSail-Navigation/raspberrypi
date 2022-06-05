import time
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
from nav_algo.events import Events
from nav_algo.navigation_helper import *
# from nav_algo.camera import Camera


class NavigationController:
    """A controller class for the navigation algorithm.

    Args:
        waypoints (list of (float, float)): A list of (latitude, longitude) tuples of waypoints.

    Attributes:
        DETECTION_RADIUS (float): How close we need to get to a waypoint.
        coordinate_system (CoordinateSystem): The global coordinate system.
        waypoints (list of Vector): Position vectors of waypoints.
        boat (BoatController): A representation of the boat.
        radio (Radio): Prints navigation data to the base station.
        current_waypoint (Vector): The current target waypoint.
        boat_position (Vector): The current position of the boat.
        boat_to_target (Vector): The vector from the boat to the target position.
        simulation (bool): If we are running a simulation

    """
    def __init__(self,
                 event=Events.FLEET_RACE,
                 waypoints=[],
                 simulation=False):

        # TODO this is the hacky way of doing this. At some point, someone should fix it
        # so that you can actually get sensor data and stuff - CM
        # If the event is fleet race, we don't care about the algo, just set angles
        # NOTE commands should end with \n, send 'q' to quit, angles are space delineated 'main tail'
        if event == Events.FLEET_RACE:
            self.boat = boat.BoatController()
            self.radio = radio.Radio(9600, self.boat, True, t=10)
            self.radio.transmitString(
                "Starting Fleet Race\nSend angles of the form 'sail_angle rudder_angle'"
            )
            while True:
                try:
                    self.radio.receiveString()  # timeout is 1 sec
                except:
                    pass

        self.DETECTION_RADIUS = 5.0

        self.coordinate_system = coord.CoordinateSystem(
            waypoints[0][0], waypoints[0][1])
        self.waypoints = [
            coord.Vector(self.coordinate_system, w[0], w[1]) for w in waypoints
        ]

        self.boat = boat.BoatController(
            coordinate_system=self.coordinate_system)

        self.radio = radio.Radio(9600)
        self.radio.transmitString("Waiting for GPS fix...\n")

        # wait until we know where we are
        while self.boat.sensors.velocity is None:
            self.boat.sensors.readGPS()  # ok if this is blocking

        self.radio.transmitString(
            "Established GPS fix. Beginning navigation...\n")
        # self.current_waypoint = self.waypoints.pop(0)
        # self.current_waypoint = self.waypoints[desired_fst_waypoint]
        # TODO: add modified ^ to event algos before each navigate call

        if event == Events.ENDURANCE:
            # 7 hrs = 25200 sec
            exit_before = 25200
            start_time = time.time()
            loop_waypoints = endurance(self.waypoints, opt_dist=10, offset=10)
            self.current_waypoint = loop_waypoints[3]
            while (time.time() - start_time < exit_before):
                self.waypoints = loop_waypoints
                self.current_waypoint = self.waypoints.pop(0)
                self.navigate
        elif event == Events.STATION_KEEPING:
            # to find an optimal radius, 10 for now
            exit_before = 300
            circle_radius = 10
            self.waypoints = stationKeeping(self.waypoints, circle_radius,
                                            "ENTRY")
            self.current_waypoint = self.waypoints.pop(0)
            self.navigate()
            # Set timer
            start_time = time.time()
            loop_waypoints = stationKeeping(self.waypoints, circle_radius,
                                            "KEEP")
            while time.time() - start_time < exit_before:
                self.waypoints = loop_waypoints
                self.current_waypoint = self.waypoints.pop(0)
                self.navigate()
            self.waypoints = stationKeeping(self.waypoints, circle_radius,
                                            "EXIT")
        elif event == Events.PRECISION_NAVIGATION:
            self.waypoints = precisionNavigation(self.waypoints)
        elif event == Events.COLLISION_AVOIDANCE:
            self.waypoints = collisionAvoidance(self.waypoints)
            self.current_waypoint = self.waypoints[0]
            self.navigateDetection()
        elif event == Events.SEARCH:
            self.waypoints = search(self.waypoints)
            self.current_waypoint = self.waypoints[0]
            self.navigateDetection(event=Events.SEARCH)

        self.current_waypoint = self.waypoints.pop(0)
        self.navigate()

        # TODO Clean up ports
        self.radio.serialStream.close()

    def navigate(self):
        """ Execute the navigation algorithm.

        This is a blocking call that runs until all waypoints have been hit.

        """
        while self.current_waypoint is not None:
            # read for a quit signal ('q') or manual override ('o')
            try:
                self.radio.readline()
            except:
                pass

            while self.radio.fleetRace:
                # manual override has been engaged, wait for autopilot signal ('a')
                try:
                    self.radio.readline()
                except:
                    pass

            all_waypts = [pt for pt in self.waypoints]
            all_waypts.append(self.current_waypoint)
            self.radio.printAllWaypoints(all_waypts)
            time.sleep(4)  # TODO how often should this run?

            self.boat.updateSensors()
            self.boat_position = self.boat.getPosition()
            self.radio.printData(self.boat)

            if self.boat_position.xyDist(
                    self.current_waypoint) < self.DETECTION_RADIUS:
                # hit waypoint -- send data back to basestation
                self.radio.printHitWaypoint(self.current_waypoint)

                if len(self.waypoints) > 0:
                    self.current_waypoint = self.waypoints.pop(0)
                else:
                    self.current_waypoint = None
                    break

            sailing_angle = newSailingAngle(self.boat, self.current_waypoint)
            self.boat.setServos(sailing_angle)

    def navigateDetection(self, event=Events.COLLISION_AVOIDANCE):
        # TODO: modify to implement collision avoidance
        while self.current_waypoint is not None:
            time.sleep(2)

            self.boat.updateSensors()
            self.boat_position = self.boat.getPosition()
            (buoy_coords, obst_coords) = Camera.read(self.boat.sensors.yaw,
                                                     self.boat_position.x,
                                                     self.boat_position.y)
            if (buoy_coords is not None & event == Events.SEARCH):
                # TODO: get buoy pos (buoy_waypoint)
                buoy_coords = coord.Vector(x=buoy_coords[0], y=buoy_coords[1])
                self.current_waypoint = buoy_coords
                self.waypoints = [buoy_coords]

            if (obst_coords is not None):
                obstacle_pos1 = obst_coords
                # TODO: get obstacle_pos at time t
                time.sleep(2)
                snd_read = Camera.read(self.boat.sensors.yaw,
                                       self.boat_position.x,
                                       self.boat_position.y)
                obstacle_pos2 = snd_read[0]
                avoidance_waypoint = assessCollision(obstacle_pos1,
                                                     obstacle_pos2, 2)
                if avoidance_waypoint is not None:
                    avoidance_waypoint = coord.Vector(x=avoidance_waypoint[0],
                                                      y=avoidance_waypoint[1])
                    self.current_waypoint = avoidance_waypoint
                    self.waypoints.insert(0, avoidance_waypoint)

            else:
                if self.boat_position.xyDist(
                        self.current_waypoint) < self.DETECTION_RADIUS:
                    if len(self.waypoints) > 1:
                        self.current_waypoint = self.waypoints[1]
                        del (self.waypoints[0])
                    else:
                        self.current_waypoint = None
                        del (self.waypoints[0])
                        break

            sailing_angle = newSailingAngle(self.boat, self.current_waypoint)
            self.boat.setServos(sailing_angle)
