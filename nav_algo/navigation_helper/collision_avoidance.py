from navigation import *

def collision_avoidance(NavigationController):
    NavigationController.camera = Camera()

    # Get waypoints between the input buoys
    NavigationController.configuration.waypoints = collisionAvoidance(
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


def collisionAvoidance(buoy_waypoints, boat):
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