from nav_algo.event_helper.station_keeping_event.station_keeping_helpers import stationKeepingHelper
from nav_algo.coordinates import Vector, CoordinateSystem

class MimicBoat:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def getPosition(self):
        return Vector(x=self.x, y=self.y)

# myBoat = MimicBoat(0, 90)

nw = Vector(x=0, y=100)
ne = Vector(x=100, y=100)
se = Vector(x=100, y=0)
sw = Vector(x=0, y=0)
buoy_waypoints = [nw, ne, se, sw]

#returned_waypoints = stationKeepingHelper(buoy_waypoints, 10, "ENTRY", boat=myBoat)
# for v in returned_waypoints:
#     print('(' + str(v.x) + ', ' +  str(v.y) + ')')

class StationKeepingTests:
    def entryStationKeepingTest(self, expected_waypoint, boat_x, boat_y):
        boat = MimicBoat(boat_x, boat_y)
        returned_waypoints = stationKeepingHelper(buoy_waypoints, 10, "ENTRY", boat=boat)
        return returned_waypoints[0].x == expected_waypoint.x and returned_waypoints[0].y == expected_waypoint.y
    
    # def exitStationKeepingTest(self, expected_waypoint, boat_x, boat_y):


        


