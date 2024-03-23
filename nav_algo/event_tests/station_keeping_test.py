from nav_algo.event_helper.station_keeping_event.station_keeping_helpers import stationKeepingHelper
from nav_algo.coordinates import Vector

class MimicBoat:
    def getPosition(self):
        return Vector(x=50, y=50)

myBoat = MimicBoat()


nw = Vector(x=0, y=100)
ne = Vector(x=100, y=100)
se = Vector(x=100, y=0)
sw = Vector(x=0, y=0)
buoy_waypoints = [nw, ne, se, sw]


returned_waypoints = stationKeepingHelper(buoy_waypoints, 10, "ENTRY", boat=myBoat)
for v in returned_waypoints:
    print('(' + str(v.x) + ', ' +  str(v.y) + ')')



