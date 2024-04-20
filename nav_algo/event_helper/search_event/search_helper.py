#from nav_algo.navigation import *
#from nav_algo import sensors
import math
import random
from nav_algo.coordinates import Vector

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
    print(center_point)
    x = center_point[0] - 100
    for i in range(-radius, radius, 1):
        y = center_point[1] + fcn(x, 100)
        new_waypoint = Vector(x=x, y=y)
        search_waypoints.insert(len(search_waypoints), new_waypoint)
        x += 1
    # Rotating waypoints based on initial wind direction
    #theta = sensors.readWindDirection()
    theta = random.randint(1, 360)
    for point in search_waypoints:
        rotatedx = point.x*math.cos(theta) - point.y*math.sin(theta)
        rotatedy = point.x*math.sin(theta) + point.y*math.cos(theta)
        new_rotated_waypoint = Vector(x=rotatedx, y=rotatedy)
        rotated_waypoints.insert(len(rotated_waypoints), new_rotated_waypoint)
    #print(rotated_waypoints)
    return rotated_waypoints

def fcn(x, radius):
    """
    Creates path for sailboat and then rotates this path based on the wind 
    angle before the boat enters the circle
    """
    period = 1
    return math.sqrt((radius**2)-(x**2))*math.sin(period*x*(math.pi))