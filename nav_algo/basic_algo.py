import nav_algo.boat as b
from nav_algo.navigation_helper import *
from navigation_utilities import getServoAnglesImpl
    
class BasicAlgo:
    """
    The old navigation algorithm - this currently sucks and needs a major
    refactor if it's ever going to be used.
    """
    def __init__(self):
        pass
    
    def step(self, boat : b.BoatController, waypoint):
        intended_angle = newSailingAngle(boat, waypoint)

        abs_wind_dir = boat.sensors.wind_direction # TODO is this right?
        yaw = boat.sensors.yaw
        sail, rudder = getServoAnglesImpl(abs_wind_dir, yaw, intended_angle)
        return sail, rudder