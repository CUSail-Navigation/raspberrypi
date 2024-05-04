from nav_algo.navigation import Camera, time, coord
from nav_algo.navigation_utilities import collisionAvoidanceImpl, midpoint
import math 

def collision_avoidance(NavigationController):
    NavigationController.camera = Camera()

    NavigationController.configuration.waypoints = collisionAvoidanceWaypoints(
        NavigationController.configuration.waypoints)
    NavigationController.current_waypoint = NavigationController.configuration.waypoints.pop(0)

    # Navigate until we see a boat
    NavigationController.navigate(use_camera=True)
    boat_loc = "DETECTION ALGO"
    while boat_loc is not None:
        NavigationController.configuration.write_output(f"SEE BOAT FROM {boat_loc}")
        NavigationController.configuration.waypoints.push(NavigationController.current_waypoint)
        waypoint_x, waypoint_y = NavigationController.current_waypoint
        curr_x, curr_y = NavigationController.configuration.boat.getPosition()
        if boat_loc == "right":
            # set destination 90 degrees to the right
            dx = waypoint_x - curr_x
            dy = waypoint_y - curr_y
            NavigationController.current_waypoint = coord.Vector(x = curr_x + dy, y = curr_y - dx)
        else:
            # set destination 90 degrees to the left
            dx = waypoint_x - curr_x
            dy = waypoint_y - curr_y
            NavigationController.current_waypoint = coord.Vector(x = curr_x - dy, y = curr_y + dx)
        time.sleep(10.0)
        NavigationController.navigate(use_camera=True)


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