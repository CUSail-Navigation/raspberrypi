import nav_algo.coordinates as coord
import nav_algo.boat as boat
import math
import numpy as np


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


def endurance(waypoints, opt_dist, offset):
    # waypoints in a clockwise order, starting with top left
    factor = opt_dist / math.sqrt(2)
    first = (waypoints[0].x - factor, waypoints[0].y + factor)
    second = (waypoints[1].x + factor, waypoints[1].y + factor)
    third = (waypoints[2].x + factor, waypoints[2].y - factor)
    fourth = (waypoints[3].x - factor, waypoints[3].y - factor)
    # waypoints placed between corners
    midpoint1 = first.midpoint(second)
    first_second = (midpoint1.x, midpoint1.y+offset)
    midpoint2 = second.midpoint(third)
    second_third = (midpoint2.x + offset, midpoint2.y)
    midpoint3 = third.midpoint(fourth)
    third_fourth = (midpoint1.x, midpoint1.y-offset)
    midpoint4 = fourth.midpoint(first)
    fourth_first = (midpoint4.x - offset, midpoint4.y)
    return [first, first_second, second, second_third,
            third, third_fourth, fourth, fourth_first]


def stationKeeping(waypoints, circle_radius, state, opt_angle=45):
    if state == "ENTRY":
        stationKeepingWaypoints = []  # Necessary waypoints
        # entry point to the square
        square_entries = [waypoints[0].midpoint(waypoints[1]),
                          waypoints[1].midpoint(waypoints[2]),
                          waypoints[2].midpoint(waypoints[3]),
                          waypoints[3].midpoint(waypoints[0])]
        curr_pos = boat.getPosition()
        shortest_dist = min((curr_pos.xyDist(square_entries[0]), square_entries[0]),
                            (curr_pos.xyDist(
                                square_entries[1]), square_entries[1]),
                            (curr_pos.xyDist(
                                square_entries[2]), square_entries[2]),
                            (curr_pos.xyDist(
                                square_entries[3]), square_entries[3]),
                            key=lambda x: x[0])
        stationKeepingWaypoints.append(shortest_dist[1])

        # center of the sqaure
        center = waypoints[0].midpoint(waypoints[2])
        stationKeepingWaypoints.append(center)
        # waypoints = stationKeepingWaypoints
        return stationKeepingWaypoints
    elif state == "KEEP":
        # downwind=wind-yaw=0=clockwise,
        keep_waypoints = []
        # radian_angle = math.radians(opt_angle)
        x_coord = boat.getPosition().x
        y_coord = boat.getPosition().y
        boat_direction = boat.sensors.yaw
        # get wind direction relative to boat
        relative_wind = boat.sensors.wind_direction-boat_direction
        if relative_wind < 0:
            relative_wind += 360
        if relative_wind >= 180:
            # move ccw
            loop_direction = 1
        else:
            # move cw
            loop_direction = -1
        # compute angle of first waypoint
        first_angle = boat_direction + opt_angle * loop_direction
        if (first_angle < 0):
            first_angle += 360
        # convert to radians for computation of other waypoints using trig
        first_angle_rad = math.radians(first_angle)

        for i in range(4):
            # place 4 waypoints each 90 deg apart; when angle>2pi, trig functions know to shift input to be in range
            input_angle = first_angle_rad + \
                loop_direction * i * math.radian(90)
            keep_waypoints.append((x_coord+circle_radius*math.cos(input_angle)),
                                  (y_coord+circle_radius*math.sin(input_angle)))
        return keep_waypoints

    elif state == "EXIT":
        # corner waypoint order: NW, NE, SE, SW
        # TODO: ask Courtney about the units of x-y coord
        units_away = 10
        # north exit
        north_exit = waypoints[0].midpoint(waypoints[1])
        north_exit.y += units_away
        # east exit
        east_exit = waypoints[1].midpoint(waypoints[2])
        east_exit.x += units_away
        # south exit
        south_exit = waypoints[2].midpoint(waypoints[3])
        south_exit.y -= units_away
        # west exit
        west_exit = waypoints[0].midpoint(waypoints[3])
        west_exit.x -= units_away
        # exit waypoint order in list: N, E, S, W
        curr_pos = boat.getPosition()
        shortest_dist = min((curr_pos.xyDist(north_exit), north_exit),
                            (curr_pos.xyDist(east_exit), east_exit),
                            (curr_pos.xyDist(south_exit), south_exit),
                            (curr_pos.xyDist(west_exit), west_exit),
                            key=lambda x: x[0])
        return [shortest_dist[1]]


def find_inner_outer_points(start_point, end_point, dist, flag):
    # 1 is in, -1 is out, 0 is on the line
    slope = (start_point.y - end_point.y) / (start_point.x - end_point.x)
    theta_slope = math.atan(slope)
    x = start_point.x + dist * math.cos(theta_slope)
    y = start_point.y + dist * math.sin(theta_slope)
    if flag != 0:
        x += -flag * 0.1 * dist * math.sin(theta_slope)
        y += flag * 0.1 * dist * math.cos(theta_slope)
    return (x, y)


def buoy_offset(start_point, buoy, dist):
    slope = (start_point.y - end_point.y) / (start_point.x - end_point.x)
    theta_slope = math.atan(slope)
    x = buoy.x + dist * math.cos(theta_slope)
    y = buoy.y + dist * math.sin(theta_slope)
    return (x, y)


def precisionNavigation(waypoints, offset=5.0, side_length=50.0):
    # waypoints:[topleft_buoy, topright_buoy, botleft_buoy, botright_buoy]
    topleft_buoy = waypoints[0]
    topright_buoy = waypoints[1]
    botleft_buoy = waypoints[2]
    botright_buoy = waypoints[3]

    x_coord = boat.getPosition().x
    y_coord = boat.getPosition().y
    boat_direction = boat.sensors.yaw
    relative_wind = boat.sensors.wind_direction-boat_direction
    if relative_wind < 0:
        relative_wind += 360
    if relative_wind >= 180:
        # move ccw
        sail_direction = 1
    else:
        # move cw
        sail_direction = -1

    start_pos = topleft_buoy.midpoint(topright_buoy)
    # find inner and outer waypoints on first side of triangle
    start_point = start_pos
    end_point = botleft_buoy
    dist = (start_point.xyDist(end_point))/3
    point_line = find_inner_outer_points(start_point, end_point, dist, 0)
    first_waypoint = find_inner_outer_points(point_line, end_point, dist, 1)
    dist_out = 2 * dist
    point_line_2 = find_inner_outer_points(start_point, end_point, dist_out, 0)
    second_waypoint = find_inner_outer_points(
        point_line_2, end_point, dist_out, -1)
    # first offset from buoy
    dist_buoy = (start_point.xyDist(end_point))/10
    third_waypoint = buoy_offset(start_point, end_point, dist_buoy)

    # find inner and outer waypoints on second side of triangle
    start_point = end_point
    end_point = botright_buoy

    dist = (start_point.xyDist(end_point))/3
    point_line = find_inner_outer_points(start_point, end_point, dist, 0)
    fourth_waypoint = find_inner_outer_points(point_line, end_point, dist, 1)
    dist_out = 2 * dist
    point_line_2 = find_inner_outer_points(start_point, end_point, dist_out, 0)
    fifth_waypoint = find_inner_outer_points(
        point_line_2, end_point, dist_out, -1)
    # second offset from buoy
    dist_buoy = (start_point.xyDist(end_point))/10
    sixth_waypoint = buoy_offset(start_point, end_point, dist_buoy)

    # find inner and outer waypoints on third side of triangle
    start_point = end_point
    end_point = start_point
    dist = (start_point.xyDist(end_point))/3
    point_line = find_inner_outer_points(start_point, end_point, dist, 0)
    seventh_waypoint = find_inner_outer_points(point_line, end_point, dist, -1)
    dist_out = 2 * dist
    point_line_2 = find_inner_outer_points(start_point, end_point, dist_out, 0)
    eighth_waypoint = find_inner_outer_points(
        point_line_2, end_point, dist_out, 1)
    # final waypoint is start_point
    ninth_waypoint = start_pos

    waypoints = [first_waypoint, second_waypoint, third_waypoint,
                 fourth_waypoint, fifth_waypoint, sixth_waypoint,
                 seventh_waypoint, eighth_waypoint, ninth_waypoint]
    return


def getRectangleBox(center, theta):
    """
    Returns rectangular box given center point and direction
    Corners ordered ccw starting from front left
    """
    r = math.sqrt(5) / 2
    corner_one = (center.x + r * (math.cos(theta + math.atan(0.5))),
                  center.y + r * (math.sin(theta + math.atan(0.5))))

    corner_two = (center.x + r * (math.cos(theta + math.pi - math.atan(0.5))),
                  center.y + r * (math.sin(theta + math.pi - math.atan(0.5))))

    corner_three = (center.x + r * (math.cos(theta - math.pi + math.atan(0.5))),
                    center.y + r * (math.sin(theta - math.pi + math.atan(0.5))))

    corner_four = (center.x + r * (math.cos(theta - math.atan(0.5))),
                   center.y + r * (math.sin(theta - math.atan(0.5))))
    return [corner_one, corner_two, corner_three, corner_four]


def check_overlap(box_coords, obst_coords, axis):
    box_list = []
    obst_list = []
    for index in range(len(box_coords)):
        box_prod = box_coords[index] @ axis
        box_list.append(box_prod)
        obst_prod = obst_coords[index] @ axis
        obst_list.append(obst_prod)
    if min(obst_list) > max(box_list) or min(box_list) > max(obst_list):
        return False
    else:
        return True


def isCollision(boat_coords, obst_coords):
    """
    Returns True if boat_coords and obst_coords overlap, else False
    """
    boat_vecs = []
    for boat_coord in boat_coords:
        vec = np.array([boat_coord[0], boat_coord[1]])
        boat_vecs.append(vec)

    obst_vecs = []
    for obst_coord in obst_coords:
        vec = np.array([obst_coord[0], obst_coord[1]])
        obst_vecs.append(vec)

    boat_front_norm = boat_vecs[0] - boat_vecs[1]
    boat_side_norm = boat_vecs[0] - boat_vecs[3]

    obst_front_norm = obst_vecs[0] - obst_vecs[1]
    obst_side_norm = obst_vecs[0] - obst_vecs[3]

    if (check_overlap(boat_vecs, obst_vecs, boat_front_norm) and
        check_overlap(boat_vecs, obst_vecs, boat_side_norm) and
        check_overlap(boat_vecs, obst_vecs, obst_front_norm) and
            check_overlap(boat_vecs, obst_vecs, obst_side_norm)):
        return True
    else:
        return False


def getVelocity(point_before, point_after, t):
    """
    Returns velocity of object during time interval t as [speed, theta]

    point_before: center of object at time 0
    point_after: center of object at time t
    t: time interval
    """
    if t <= 0:
        return [0, 0]
    x_dist = point_after.x-point_before.x
    y_dist = point_after.y-point_before.y
    total_dist = (x_dist**2+y_dist**2)**0.5
    speed = total_dist / t
    theta = math.atan(y_dist / x_dist)
    return [speed, theta]


def collisionWaypoint(time):
    """
    Returns waypoint 2m away to avoid obstacle detected at time time
    """
    angle_deg = (boat.sensors.yaw + 90 - time)
    while (angle_deg >= 360.0):
        angle_deg -= 360.0
    angle_rad = math.radians(angle_deg)
    return (boat.getPosition().x + 2 * math.cos(angle_rad),
            boat.getPosition().y + 2 * math.sin(angle_rad))


def assessCollision(obst_point, obst_point_2, time)
  """
    Checks if collision occurs. Returns new waypoint if collision, else None

    obst_point: position of obstacle at time 0
    obst_point_2: position of obstacle at time time
    time: time between detection of obst_point and obst_point_2
    """
   [obst_speed, obst_theta] =
      getVelocity(obstacle_point, obstacle_point_2, time)
    collision_time = 0
    while (collision_time <= 15):
        new_obstacle_point =
          (obstacle_point.x + i * obst_speed * math.cos(obst_theta),
            obstacle_point.y + i * obst_speed * math.sin(obst_theta))
        new_obstacle_box = getRectangleBox(new_obstacle_point, obst_theta)
        new_boat_box =
          (boat.getPosition().x + i * boat.sensors.velocity *
            math.cos(math.radians(boat.sensors.yaw)),
            boat.getPosition().y + i * boat.sensors.velocity *
            math.sin(math.radians(boat.sensors.yaw)))
        if isCollision(new_boat_box, new_obstacle_box):
            return collisionWaypoint(collision_time)
        else:
            collision_time += 1


def unitVector(coords):
    magnitude = math.sqrt(coords[0]**2 + coords[1]**2)
    return (coords[0] / magnitude, coords[1] / magnitude)


def collisionAvoidance(buoy_waypoint):
    """
    Returns waypoints list for ideal path (no obstacles)
    """
    orientation = (buoy_waypoints[0] - boat.getPosition().x,
                   buoy_waypoints[1] - boat.getPosition().y)
    first_direction = (orientation[1], -1 * orientation[0])
    first_direction = unitVector(first_direction)
    dist = 10
    first_waypoint = (buoy_waypoints[0] + dist * first_direction[0],
                      buoy_waypoints[1] + dist * first_direction[1])
    second_direction = unitVector(orientation)
    second_waypoint = (buoy_waypoints[0] + dist * second_direction[0],
                       buoy_waypoints[1] + dist * second_direction[1])
    third_direction = (-1 * orientation[1], orientation[0])
    third_direction = unitVector(third_direction)
    third_waypoint = (buoy_waypoints[0] + dist * third_direction[0],
                      buoy_waypoints[1] + dist * third_direction[1])
    fourth_waypoint = (boat.getPosition().x + dist * second_direction[0],
                       boat.getPosition().y + dist * second_direction[1])
    return [first_waypoint, second_waypoint, third_waypoint, fourth_waypoint,
            boat.getPosition()]


def search(center_point, state, scalar=3, constant=(100-30*math.pi)):
    if state == "SEARCH":
        i = 0
        for theta in range (40,-1,-1):
            t = (theta * math.pi / 4)
            r = constant + scalar * t
            x = center_point[0] + r * math.cos(t)
            y = center_point[1] + r * math.sin(t)
            waypoints[i] = (x, y)
            i += 1
        return waypoints
    elif state == "FOUND":
        pass
