import time
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio


class NavigationController:
    """A controller class for the navigation algorithm.

    Args:
        waypoints (list of (float, float)): A list of (latitude, longitude) tuples of waypoints.
        simulation (bool): (Optional) A parameter used to bypass blocking calls.

    Attributes:
        DETECTION_RADIUS (float): How close we need to get to a waypoint.
        BEATING (float): The beating parameter (width of a beating path).
        DELTA_ALPHA (float): The smallest heading angle change we can control.
        coordinate_system (CoordinateSystem): The global coordinate system.
        waypoints (list of Vector): Position vectors of waypoints.
        boat (BoatController): A representation of the boat.
        radio (Radio): Prints navigation data to the base station.
        current_waypoint (Vector): The current target waypoint.
        boat_position (Vector): The current position of the boat.
        boat_to_target (Vector): The vector from the boat to the target position.

    """
    def __init__(self, waypoints, simulation=False):
        self.DETECTION_RADIUS = 5.0
        self.BEATING = 10.0
        self.DELTA_ALPHA = 1.0

        self.coordinate_system = coord.CoordinateSystem(
            waypoints[0][0], waypoints[0][1])
        self.waypoints = [
            coord.Vector(self.coordinate_system, w[0], w[1]) for w in waypoints
        ]

        self.boat = boat.BoatController()

        # may want to bypass this if testing
        self.radio = radio.Radio()
        self.radio.transmitString("Waiting for GPS fix...\n")

        if not simulation:
            # wait until we know where we are
            while not self.boat.sensors.fix:
                self.boat.sensors.readGPS()  # ok if this is blocking

        self.radio.transmitString(
            "Established GPS fix. Beginning navigation...\n")

        self.current_waypoint = self.waypoints.pop(0)

    def navigate(self):
        """ Execute the navigation algorithm.

        This is a blocking call that runs until all waypoints have been hit.

        """
        while self.current_waypoint is not None:
            time.sleep(2)  # TODO how often should this run?

            self.boat.updateSensors()
            self.boat_position = self.boat.getPosition(self.coordinate_system)

            if self.boat_position.xyDist(
                    self.current_waypoint) < self.DETECTION_RADIUS:
                if len(self.waypoints) > 0:
                    self.current_waypoint = self.waypoints.pop(0)
                else:
                    self.current_waypoint = None
                    break

            self.boat_to_target = self.current_waypoint.vectorSubtract(
                self.boat_position)

            sailing_angle = self.newSailingAngle()
            self.boat.setServos(sailing_angle)

        # TODO cleanup pins?

    def newSailingAngle(self):
        """Determines the best angle to sail at.

        The sailboat follows a locally optimal path (maximize vmg while minimizing
        directional changes) until the global optimum is "better" (based on the
        hysterisis factor).

        Returns:
            float: The best angle to sail (in the global coordinate system).

        """
        right_angle_max, right_vmg_max = self.optAngle(True)
        left_angle_max, left_vmg_max = self.optAngle(False)

        boat_heading = self.boat.sensors.velocity.angle()
        hysterisis = 1.0 + (self.BEATING / self.boat_to_target.magnitude())
        sailing_angle = right_angle_max
        if (abs(right_angle_max - boat_heading) <
                abs(left_angle_max - boat_heading)
                and right_vmg_max * hysterisis < left_vmg_max) or (
                    abs(right_angle_max - boat_heading) >=
                    abs(left_angle_max - boat_heading)
                    and left_vmg_max * hysterisis >= right_vmg_max):
            sailing_angle = left_angle_max

        return sailing_angle

    def optAngle(self, right):
        """Determines the best angle to sail on either side of the wind.

        The "best angle" maximizes the velocity made good toward the target.

        Args:
            right (bool): True if evaluating the right side of the wind, False for left.

        Returns:
            float: The best angle to sail (in the global coordinate system).
            float: The velocity made good at the best angle.

        """
        alpha = 0.0
        best_vmg = 0.0
        wind_angle = self.boat.sensors.wind_direction
        best_angle = wind_angle

        while alpha < 180:
            vel = self.polar(alpha if right else -1.0 * alpha)
            vmg = vel.dot(self.boat_to_target.toUnitVector())

            if vmg > best_vmg:
                best_vmg = vmg
                best_angle = wind_angle + alpha if right else wind_angle - alpha

            alpha = alpha + self.DELTA_ALPHA

        return coord.rangeAngle(best_angle), best_vmg

    def polar(self, angle):
        """Evaluates the polar diagram for a given angle relative to the wind.

        Args:
            angle (float): A potential boat heading relative to the absolute wind direction.

        Returns:
            Vector: A unit boat velocity vector in the global coordinate system.

        """
        # TODO might want to use a less simplistic polar diagram
        angle = coord.rangeAngle(angle)
        if (angle > 20 and angle < 160) or (angle > 200 and angle < 340):
            # put back into global coords
            return coord.Vector(angle=angle + self.boat.sensors.wind_direction)
        return coord.Vector.zeroVector()
