from nav_algo.navigation import *

def searchHelper(waypoints):
    """
    Requires: waypoints[0] must be the center point of the circle. Returns 
    waypoints for the search path depending on the initial wind direction. 
    """
    search_waypoints = []
    rotated_waypoints = []
    center_point = waypoints[0]
    radius = 100
    # Generating waypoints
    x = center_point.x - 100
    for i in range(-radius, radius, 1):
        y = center_point.y + fcn(x, 100)
        new_waypoint = coord.Vector(x, y)
        search_waypoints.insert(len(search_waypoints), new_waypoint)
        x += 1
    # Rotating waypoints based on initial wind direction
    theta = sensors.readWindDirection()
    for point in search_waypoints:
        rotatedx = point.x*math.cos(theta) - point.y*math.sin(theta)
        rotatedy = point.x*math.sin(theta) + point.y*math.cos(theta)
        new_rotated_waypoint = coord.Vector(rotatedx, rotatedy)
        rotated_waypoints.insert(len(rotated_waypoints), new_rotated_waypoint)
    return rotated_waypoints