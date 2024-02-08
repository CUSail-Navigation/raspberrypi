import nav_algo.coordinates as coord
import nav_algo.navigation_utilities as util
import math
import numpy as np
import sensors


def newSailingAngle(boat, target):
    """TODO Determines the best angle to sail at.

        The sailboat follows a locally optimal path (maximize vmg while minimizing
        directional changes) until the global optimum is "better" (based on the
        hysterisis factor).

        Returns:
            float: The best angle to sail (in the global coordinate system).

    """
    boat_position = boat.getPosition()
    boat_position = (boat_position.x, boat_position.y)

    target_position = (target.x, target.y)
    angle_boat_heading = boat.sensors.velocity.angle()
    abs_wind_dir = boat.sensors.wind_direction
    return util.newSailingAngleImpl(boat_position, target_position,
                                    angle_boat_heading, abs_wind_dir)


# def optAngle(boat_to_target, boat, right):
#     """Determines the best angle to sail on either side of the wind.

#         The "best angle" maximizes the velocity made good toward the target.

#         Args:
#             right (bool): True if evaluating the right side of the wind, False for left.

#         Returns:
#             float: The best angle to sail (in the global coordinate system).
#             float: The velocity made good at the best angle.

#     """
#     return util.optAngleImpl(boat_to_target.angle(),
#                              boat.sensors.wind_direction, right)


def polar(angle, boat):
    """Evaluates the polar diagram for a given angle relative to the wind.

        Args:
            angle (float): A potential boat heading relative to the absolute wind direction.
            boat (BoatController): The BoatController (either sim or real)

        Returns:
            Vector: A unit boat velocity vector in the global coordinate system.

      """
    # TODO might want to use a less simplistic polar diagram
    x, y = util.polarImpl(angle, boat.sensors.wind_direction)
    return coord.Vector(x=x, y=y)


# def counterClockwiseRect(waypoints, boat, buoy_offset=2):
#     if len(waypoints) > 4:
#         raise RuntimeError('More than four waypoints given for endurance')

#     # classify the waypoints as upper right, upper left, lower left, lower right
#     # in increasing order of angle of the line from the center of the square to
#     # the waypoint
#     center_x = sum([w.x for w in waypoints]) / len(waypoints)
#     center_y = sum([w.y for w in waypoints]) / len(waypoints)
#     center = coord.Vector(x=center_x, y=center_y)

#     disps = [w.vectorSubtract(center) for w in waypoints]
#     angles = [d.angle() for d in disps]

#     wa = []
#     for i in range(len(angles)):
#         wa.append((waypoints[i], angles[i]))
#     wa.sort(key=lambda x: x[1])

#     # get labeled buoys
#     ur = wa[0][0]
#     ul = wa[1][0]
#     ll = wa[2][0]
#     lr = wa[3][0]

#     # get the angle of the line between ll and ur and the offset waypoints
#     theta = np.arctan2(ur.y - ll.y, ur.x - ll.x)
#     w_ll = coord.Vector(x=ll.x - buoy_offset * np.cos(theta),
#                         y=ll.y - buoy_offset * np.sin(theta))
#     w_ur = coord.Vector(x=ur.x + buoy_offset * np.cos(theta),
#                         y=ur.y + buoy_offset * np.sin(theta))

#     # get the angle of the line between lr and ul and the offset waypoints
#     phi = np.arctan2(ul.y - lr.y, ul.x - ll.x)
#     w_lr = coord.Vector(x=lr.x - buoy_offset * np.cos(phi),
#                         y=lr.y - buoy_offset * np.sin(phi))
#     w_ul = coord.Vector(x=ul.x + buoy_offset * np.cos(phi),
#                         y=ul.y + buoy_offset * np.sin(phi))

#     # put the waypoints into order starting from the closest to the boat in
#     # a counter-clockwise direction
#     boat_pos = boat.getPosition()
#     boat_angle = boat_pos.vectorSubtract(center).angle()

#     if boat_angle > wa[3][1] or boat_angle < wa[0][1]:
#         return [w_ur, w_ul, w_ll, w_lr]
#     elif boat_angle > wa[2][1]:
#         return [w_lr, w_ur, w_ul, w_ll]
#     elif boat_angle > wa[1][1]:
#         return [w_ll, w_lr, w_ur, w_ul]
#     else:
#         return [w_ul, w_ll, w_lr, w_ur]


# def stationKeeping(waypoints, circle_radius, state, boat, opt_angle=45):
#     if state == "ENTRY":
#         stationKeepingWaypoints = []  # Necessary waypoints
#         # entry point to the square
#         square_entries = [
#             waypoints[0].midpoint(waypoints[1]),
#             waypoints[1].midpoint(waypoints[2]),
#             waypoints[2].midpoint(waypoints[3]),
#             waypoints[3].midpoint(waypoints[0])
#         ]
#         curr_pos = boat.getPosition()
#         shortest_dist = min(
#             (curr_pos.xyDist(square_entries[0]), square_entries[0]),
#             (curr_pos.xyDist(square_entries[1]), square_entries[1]),
#             (curr_pos.xyDist(square_entries[2]), square_entries[2]),
#             (curr_pos.xyDist(square_entries[3]), square_entries[3]),
#             key=lambda x: x[0])
#         stationKeepingWaypoints.append(shortest_dist[1])

#         # center of the square
#         center = waypoints[0].midpoint(waypoints[2])
#         stationKeepingWaypoints.append(center)
#         return stationKeepingWaypoints

#     elif state == "KEEP":
#         # downwind=wind-yaw=0=clockwise,
#         keep_waypoints = []
#         # radian_angle = math.radians(opt_angle)
#         x_coord = boat.getPosition().x
#         y_coord = boat.getPosition().y
#         boat_direction = boat.sensors.yaw
#         # get wind direction relative to boat
#         relative_wind = boat.sensors.wind_direction - boat_direction
#         if relative_wind < 0:
#             relative_wind += 360
#         if relative_wind >= 180:
#             # move ccw
#             loop_direction = 1
#         else:
#             # move cw
#             loop_direction = -1
#         # compute angle of first waypoint
#         first_angle = boat_direction + opt_angle * loop_direction
#         if (first_angle < 0):
#             first_angle += 360
#         # convert to radians for computation of other waypoints using trig
#         first_angle_rad = math.radians(first_angle)

#         for i in range(4):
#             # place 4 waypoints each 90 deg apart; when angle>2pi, trig functions know to shift input to be in range
#             input_angle = first_angle_rad + \
#                 loop_direction * i * math.radians(90)
#             keep_waypoints.append(
#                 (x_coord + circle_radius * math.cos(input_angle)),
#                 (y_coord + circle_radius * math.sin(input_angle)))
#         return keep_waypoints

#     elif state == "EXIT":
#         # TODO this assumes that the buoys are cardinal aligned, but this is
#         # probably not true. I set the distance to move away from the box to
#         # be large to account for this, but in the future, this should be
#         # rewritten without that assumption.

#         # corner waypoint order: NW, NE, SE, SW
#         units_away = 40
#         # north exit
#         north_exit = waypoints[0].midpoint(waypoints[1])
#         north_exit.y += units_away
#         # east exit
#         east_exit = waypoints[1].midpoint(waypoints[2])
#         east_exit.x += units_away
#         # south exit
#         south_exit = waypoints[2].midpoint(waypoints[3])
#         south_exit.y -= units_away
#         # west exit
#         west_exit = waypoints[0].midpoint(waypoints[3])
#         west_exit.x -= units_away
#         # exit waypoint order in list: N, E, S, W
#         curr_pos = boat.getPosition()
#         shortest_dist = min((curr_pos.xyDist(north_exit), north_exit),
#                             (curr_pos.xyDist(east_exit), east_exit),
#                             (curr_pos.xyDist(south_exit), south_exit),
#                             (curr_pos.xyDist(west_exit), west_exit),
#                             key=lambda x: x[0])
#         return [shortest_dist[1]]


def find_inner_outer_points(start_point, end_point, dist, flag):
    # 1 is in, -1 is out, 0 is on the line
    slope_y = start_point.y - end_point.y
    slope_x = start_point.x - end_point.x
    theta_slope = math.atan2(slope_y, slope_x)
    x = start_point.x + dist * math.cos(theta_slope)
    y = start_point.y + dist * math.sin(theta_slope)
    if flag != 0:
        x += -flag * 0.1 * dist * math.sin(theta_slope)
        y += flag * 0.1 * dist * math.cos(theta_slope)
    return coord.Vector(x=x, y=y)


def buoy_offset(start_point, buoy, dist):
    slope_y = start_point.y - buoy.y
    slope_x = start_point.x - buoy.x
    theta_slope = math.atan2(slope_y, slope_x)
    x = buoy.x + dist * math.cos(theta_slope)
    y = buoy.y + dist * math.sin(theta_slope)
    return coord.Vector(x=x, y=y)


# def precisionNavigation(waypoints):
#     # waypoints:[topleft_buoy, topright_buoy, botleft_buoy, botright_buoy]
#     buoys = [(w.x, w.y) for w in waypoints]
#     out_waypoints = util.precisionNavigationImpl(buoys)
#     out_waypoints = [coord.Vector(x=w[0], y=w[1]) for w in out_waypoints]
#     return out_waypoints


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

    corner_three = (center.x + r *
                    (math.cos(theta - math.pi + math.atan(0.5))), center.y +
                    r * (math.sin(theta - math.pi + math.atan(0.5))))

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

    if (check_overlap(boat_vecs, obst_vecs, boat_front_norm)
            and check_overlap(boat_vecs, obst_vecs, boat_side_norm)
            and check_overlap(boat_vecs, obst_vecs, obst_front_norm)
            and check_overlap(boat_vecs, obst_vecs, obst_side_norm)):
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
    x_dist = point_after.x - point_before.x
    y_dist = point_after.y - point_before.y
    total_dist = (x_dist**2 + y_dist**2)**0.5
    speed = total_dist / t
    theta = math.atan(y_dist / x_dist)
    return [speed, theta]


def collisionWaypoint(time, boat):
    """
    Returns waypoint 2m away to avoid obstacle detected at time time
    """
    angle_deg = (boat.sensors.yaw + 90 - time)
    while (angle_deg >= 360.0):
        angle_deg -= 360.0
    angle_rad = math.radians(angle_deg)
    return coord.Vector(boat.getPosition().x + 2 * math.cos(angle_rad),
                        boat.getPosition().y + 2 * math.sin(angle_rad))


def assessCollision(obst_point, obst_point_2, time, boat):
    """
      Checks if collision occurs. Returns new waypoint if collision, else None

      obst_point: position of obstacle at time 0
      obst_point_2: position of obstacle at time time
      time: time between detection of obst_point and obst_point_2
      """
    if (obst_point is None or obst_point_2 is None):
        return None
    [obst_speed, obst_theta] = getVelocity(obst_point, obst_point_2, time)
    collision_time = 0
    while (collision_time <= 15):
        new_obstacle_point = (
            obst_point.x + collision_time * obst_speed * math.cos(obst_theta),
            obst_point.y + collision_time * obst_speed * math.sin(obst_theta))
        new_obstacle_box = getRectangleBox(new_obstacle_point, obst_theta)
        new_boat_box = (boat.getPosition().x +
                        collision_time * boat.sensors.velocity.magnitude() *
                        math.cos(math.radians(boat.sensors.yaw)),
                        boat.getPosition().y +
                        collision_time * boat.sensors.velocity.magnitude() *
                        math.sin(math.radians(boat.sensors.yaw)))
        if isCollision(new_boat_box, new_obstacle_box):
            return collisionWaypoint(collision_time)
        else:
            collision_time += 1


# def unitVector(coords):
#     magnitude = math.sqrt(coords[0]**2 + coords[1]**2)
#     return (coords[0] / magnitude, coords[1] / magnitude)


# def collisionAvoidance(buoy_waypoints, boat):
#     """
#     Returns waypoints list for ideal path (no obstacles)
#     """
#     orientation = (buoy_waypoints[0] - boat.getPosition().x,
#                    buoy_waypoints[1] - boat.getPosition().y)
#     first_direction = (orientation[1], -1 * orientation[0])
#     first_direction = unitVector(first_direction)
#     dist = 10
#     (x1, y1) = (buoy_waypoints[0] + dist * first_direction[0],
#                 buoy_waypoints[1] + dist * first_direction[1])
#     first_waypoint = coord.Vector(x=x1, y=y1)

#     second_direction = unitVector(orientation)
#     (x2, y2) = (buoy_waypoints[0] + dist * second_direction[0],
#                 buoy_waypoints[1] + dist * second_direction[1])
#     second_waypoint = coord.Vector(x=x2, y=y2)
#     third_direction = (-1 * orientation[1], orientation[0])
#     third_direction = unitVector(third_direction)
#     (x3, y3) = (buoy_waypoints[0] + dist * third_direction[0],
#                 buoy_waypoints[1] + dist * third_direction[1])
#     third_waypoint = coord.Vector(x=x3, y=y3)
#     (x4, y4) = (boat.getPosition().x + dist * second_direction[0],
#                 boat.getPosition().y + dist * second_direction[1])
#     fourth_waypoint = coord.Vector(x=x4, y=y4)
#     return [
#         first_waypoint, second_waypoint, third_waypoint, fourth_waypoint,
#         boat.getPosition()
#     ]


# def search(waypoints):
#     """
#     Requires: waypoints[0] must be the center point of the circle. Returns 
#     waypoints for the search path depending on the initial wind direction. 
#     """
#     search_waypoints = []
#     rotated_waypoints = []
#     center_point = waypoints[0]
#     radius = 100
#     # Generating waypoints
#     x = center_point.x - 100
#     for i in range(-radius, radius, 1):
#         y = center_point.y + fcn(x, 100)
#         new_waypoint = coord.Vector(x, y)
#         search_waypoints.insert(len(search_waypoints), new_waypoint)
#         x += 1
#     # Rotating waypoints based on initial wind direction
#     theta = sensors.readWindDirection()
#     for point in search_waypoints:
#         rotatedx = point.x*math.cos(theta) - point.y*math.sin(theta)
#         rotatedy = point.x*math.sin(theta) + point.y*math.cos(theta)
#         new_rotated_waypoint = coord.Vector(rotatedx, rotatedy)
#         rotated_waypoints.insert(len(rotated_waypoints), new_rotated_waypoint)
#     return rotated_waypoints


def fcn(x, radius):
    """
    Creates path for sailboat and then rotates this path based on the wind 
    angle before the boat enters the circle
    """
    period = 1
    return math.sqrt((radius**2)-(x**2))*math.sin(period*x*(math.pi))
