import nav_algo.coordinates as coord
import nav_algo.boat as boat


def newSailingAngle(boat, target):
    """TODO Determines the best angle to sail at.

        The sailboat follows a locally optimal path (maximize vmg while minimizing
        directional changes) until the global optimum is "better" (based on the
        hysterisis factor).

        Returns:
            float: The best angle to sail (in the global coordinate system).

    """

    beating = 10.0  # TODO

    boat_to_target = target.vectorSubtract(boat.getPosition())

    right_angle_max, right_vmg_max = optAngle(boat_to_target, boat, True)
    left_angle_max, left_vmg_max = optAngle(boat_to_target, boat, False)

    boat_heading = boat.sensors.velocity.angle()
    hysterisis = 1.0 + (beating / boat_to_target.magnitude())
    sailing_angle = right_angle_max
    if (abs(right_angle_max - boat_heading) <
            abs(left_angle_max - boat_heading) and right_vmg_max * hysterisis <
            left_vmg_max) or (abs(right_angle_max - boat_heading) >=
                              abs(left_angle_max - boat_heading)
                              and left_vmg_max * hysterisis >= right_vmg_max):
        sailing_angle = left_angle_max

    return sailing_angle


def optAngle(boat_to_target, boat, right):
    """TODO Determines the best angle to sail on either side of the wind.

        The "best angle" maximizes the velocity made good toward the target.

        Args:
            right (bool): True if evaluating the right side of the wind, False for left.

        Returns:
            float: The best angle to sail (in the global coordinate system).
            float: The velocity made good at the best angle.

    """
    delta_alpha = 1.0  # TODO is this ok?
    alpha = 0.0
    best_vmg = 0.0
    wind_angle = boat.sensors.wind_direction
    best_angle = wind_angle

    while alpha < 180:
        vel = polar(alpha if right else -1.0 * alpha, boat)
        vmg = vel.dot(boat_to_target.toUnitVector())

        if vmg > best_vmg:
            best_vmg = vmg
            best_angle = wind_angle + alpha if right else wind_angle - alpha

        alpha = alpha + delta_alpha

    return coord.rangeAngle(best_angle), best_vmg


def polar(angle, boat):
    """Evaluates the polar diagram for a given angle relative to the wind.

        Args:
            angle (float): A potential boat heading relative to the absolute wind direction.
            boat (BoatController): The BoatController (either sim or real)

        Returns:
            Vector: A unit boat velocity vector in the global coordinate system.

      """
    # TODO might want to use a less simplistic polar diagram
    angle = coord.rangeAngle(angle)
    if (angle > 20 and angle < 160) or (angle > 200 and angle < 340):
        # put back into global coords
        return coord.Vector(angle=angle + boat.sensors.wind_direction)
    return coord.Vector.zeroVector()


def endurance():
    pass


# waypoints will be passed in, we can then call the navigation function whenever
# required. So first, calculate he entry point, call nacigate,
# then on empty, call station keeping back.
# 4 stages, all we need to do is calculate the waypoints and call navigate
# 0->trying to enter, 1->trying to go the center of the cicle
# 2-> looping around the circle 3-> escape
def stationKeeping(boat, square, turning_radius, timer):
    # enter the square
    # coord.CoordinateSystem(waypoints[0][0], waypoints[0][1])
    # self.waypoints = [
    #     coord.Vector(self.coordinate_system, w[0], w[1])
    #     for w in waypoints
    # ]
    square_entry = (square[0] + square[1]
                    ) / 2  #assumes first two coords represent closest side
    # center
    boat_position = boat.getPosition()
    midpoint = square[0].midpoint(square[2])
    self.current_waypoint = midpoint
    # calculate the waypoints in the circle

    # start our timer
    # loop over the circle coords
    start_time = time.time()
    circle_waypoints = []
    i = 0  #iterator
    current_waypoint = circle_waypoints[i]
    while current_waypoint is not None:
        time_elapsed = time.time() - start_time
        if time_elapsed == timer:
            break  # TODO how often should this run?
        boat.updateSensors()
        boat_position = boat.getPosition()
        if boat_position.xyDist(current_waypoint) < DETECTION_RADIUS:
            if len(self.waypoints) > 0:
                self.current_waypoint = self.waypoints.pop(0)
            else:
                self.current_waypoint = None
                break
        i += 1
        current_waypoint = circle_waypoints[i % 4]

        sailing_angle = newSailingAngle(self.boat, self.current_waypoint)
        self.boat.setServos(sailing_angle)
    # calculate the point on the square such that abs(current dist-closest dist) = min
    pass


def precisionNavigation():
    pass


def collisionAvoidance():
    pass


def search():
    pass
