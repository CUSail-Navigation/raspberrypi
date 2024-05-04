from nav_algo.navigation import Camera, time, coord
from nav_algo.navigation_utilities import collisionAvoidanceImpl, midpoint
import numpy as np
import math 

def collision_avoidance(NavigationController):
    NavigationController.camera = Camera()

    # Get waypoints between the input buoys
    NavigationController.configuration.waypoints = collisionAvoidanceWaypoints(
        NavigationController.configuration.waypoints)
    NavigationController.current_waypoint = NavigationController.configuration.waypoints.pop(0)

    # Navigate until we see a boat
    boat_loc = NavigationController.navigate(use_camera=True)
    while boat_loc is not None:
        # See the boat, push the rudder to one side to spin away for a while
        # TODO what should we really do here?
        NavigationController.configuration.write_output("SEE BOAT AT ({}, {})".format(
            boat_loc.x, boat_loc.y))
        NavigationController.configuration.boat.setServos(60.0, 20.0)
        time.sleep(10.0)  # Give time to move away
        boat_loc = NavigationController.navigate(use_camera=True)


def collisionAvoidanceWaypoints(buoy_waypoints, boat):
    """
    Returns waypoints list for ideal path (no obstacles)
    """
    orientation = (buoy_waypoints[0] - boat.getPosition().x,
                   buoy_waypoints[1] - boat.getPosition().y)
    first_direction = (orientation[1], -1 * orientation[0])
    first_direction = unitVector(first_direction)
    dist = 10
    (x1, y1) = (buoy_waypoints[0] + dist * first_direction[0],
                buoy_waypoints[1] + dist * first_direction[1])
    first_waypoint = coord.Vector(x=x1, y=y1)

    second_direction = unitVector(orientation)
    (x2, y2) = (buoy_waypoints[0] + dist * second_direction[0],
                buoy_waypoints[1] + dist * second_direction[1])
    second_waypoint = coord.Vector(x=x2, y=y2)
    third_direction = (-1 * orientation[1], orientation[0])
    third_direction = unitVector(third_direction)
    (x3, y3) = (buoy_waypoints[0] + dist * third_direction[0],
                buoy_waypoints[1] + dist * third_direction[1])
    third_waypoint = coord.Vector(x=x3, y=y3)
    (x4, y4) = (boat.getPosition().x + dist * second_direction[0],
                boat.getPosition().y + dist * second_direction[1])
    fourth_waypoint = coord.Vector(x=x4, y=y4)
    return [
        first_waypoint, second_waypoint, third_waypoint, fourth_waypoint,
        boat.getPosition()
    ]

def unitVector(coords):
    magnitude = math.sqrt(coords[0]**2 + coords[1]**2)
    return (coords[0] / magnitude, coords[1] / magnitude)
            
def collisionAvoidance(waypoints):
    # waypoints:[top two buoys, other buoy]
    buoys = [(w.x, w.y) for w in waypoints]
    out_waypoints = collisionAvoidanceImpl(buoys)
    out_waypoints = [coord.Vector(x=w[0], y=w[1]) for w in out_waypoints]
    return out_waypoints

def collisionAvoidanceImpl(buoys):
    """Generates navigation waypoints from the collision avoidance buoy locations.

        Args:
            buoys (list of (float, float)): The ordered buoy locations (start 2 buoys, then other).

        Returns:
            (list of (float, float): The generated waypoint locations.

    """
    # The first and last waypoints are between the start buoys
    mid = midpoint(buoys[0], buoys[1])
    waypoints = [mid]
    waypoints.append(buoys[2])
    waypoints.append(mid)
    return waypoints


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

# helper in assess collision
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


# used as helper in assessCollision
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

# helper in isCollision
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


# not using this
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

# helper in assess collision
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

# helper in assessCollision
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

# not used anywhere
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


            
