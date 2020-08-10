import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio


class NavigationController:
    DETECTION_RADIUS = 5.0  # how close we need to get to a waypoint (in m)
    BEATING = 30.0  # the width of the route (in m) when beating
    DELTA_ALPHA = 1.0  # the smallest change in angle that we can account for

    def __init__(self, waypoints, simulation=False):
        self.coordinate_system = coord.CoordinateSystem(
            waypoints[0][0], waypoints[0][1])
        self.waypoints = [
            coord.Vector(self.coordinate_system, w[0], w[1]) for w in waypoints
        ]

        self.boat = boat.BoatController()

        self.radio = radio.Radio()
        self.radio.transmitString("Waiting for GPS fix...\n")

        # wait until we know where we are
        while not self.sensors.fix:
            self.sensors.readGPS()  # ok if this is blocking

        self.radio.transmitString(
            "Established GPS fix. Beginning navigation...\n")

        self.current_waypoint = self.waypoints.pop(0)

    def navigate(self):
        while self.current_waypoint is not None:
            self.boat.updateSensors()
            self.boat_position = self.boat.getPosition(self.coordinate_system)

            if self.boat_position.xyDist(
                    self.current_waypoint) < self.DETECTION_RADIUS:
                self.current_waypoint = self.waypoints.pop(0)
            else:
                self.boat_to_target = self.current_waypoint.vectorSubtract(
                    self.boat_position)
                boat_heading = self.boat.sensors.velocity.angle()
                self.abs_wind = self.boat.sensors.wind_direction
                hysterisis = 1.0 + (self.BEATING /
                                    self.boat_to_target.magnitude())

                right_angle_max, right_vmg_max = self.optAngle(True)
                left_angle_max, left_vmg_max = self.optAngle(False)

                sailing_angle = right_angle_max
                if (abs(right_angle_max - boat_heading) <
                        abs(left_angle_max - boat_heading)
                        and right_vmg_max * hysterisis < left_vmg_max) or (
                            abs(right_angle_max - self.boat_heading) >=
                            abs(left_angle_max - self.boat_heading)
                            and left_vmg_max * n >= right_vmg_max):
                    sailing_angle = left_angle_max

                self.boat.setServos(sailing_angle)

    def optAngle(self, right):
        alpha = 0.0  # angle to add or subtract from current boat angle
        best_vmg = 0.0
        wind_angle = self.abs_wind.angle()
        best_angle = wind_angle

        while alpha < 180:
            vel = self.polar(alpha if right else -1.0 * alpha)
            vmg = vel.dot(self.boat_to_target.toUnitVector())

            if vmg > best_vmg:
                best_vmg = vmg
                best_angle = wind_angle + alpha if right else wind_angle - alpha

            alpha = alpha + self.DELTA_ALPHA

        return coord.rangeAngle(best_angle), best_vmg  # gives back abs angle

    def polar(self, angle):
        # TODO might want to use a less simplistic polar diagram
        angle = coord.rangeAngle(angle)
        if (angle > 20 and angle < 160) or (angle > 200 and angle < 340):
            return coord.Vector(angle=angle)
        return coord.Vector.zeroVector()
