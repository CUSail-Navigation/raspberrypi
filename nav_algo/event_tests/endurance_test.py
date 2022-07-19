import matplotlib.pyplot as plt
import sys

sys.path.append("..")
from navigation_helper import *
import coordinates as coord


class EnduranceTest:

    def __init__(self):
        print("Starting test for endurance.")

        waypoints = [(42.470370, -76.505110), (42.469246, -76.504316),
                     (42.468938, -76.505485), (42.469848, -76.506011)]

        self.coordinate_system = coord.CoordinateSystem(
            waypoints[0][0], waypoints[0][1])
        self.waypoints = [
            coord.Vector(self.coordinate_system, w[0], w[1]) for w in waypoints
        ]

        plt.figure()
        xs = [w.x for w in self.waypoints]
        ys = [w.y for w in self.waypoints]
        plt.plot(xs, ys, 'o', color="orange")

        # north
        # position = coord.Vector(self.coordinate_system,
        #                         latitude=42.470858,
        #                         longitude=-76.506220)

        # east
        # position = coord.Vector(self.coordinate_system,
        #                         latitude=42.470185,
        #                         longitude=-76.503511)

        # west
        # position = coord.Vector(self.coordinate_system,
        #                         latitude=42.469410,
        #                         longitude=-76.508731)

        # south
        position = coord.Vector(self.coordinate_system,
                                latitude=42.467748,
                                longitude=-76.504622)

        self.boat = MockBoat(position)
        plt.plot(position.x, position.y, 'r+')

        lw = self.getLoopWaypoints()
        xs = [w.x for w in lw]
        ys = [w.y for w in lw]
        colors = ['lime', 'aqua', 'indigo', 'magenta']
        labels = ['first', 'second', 'third', 'fourth']
        for i in range(len(xs)):
            plt.plot(xs[i], ys[i], 'x', color=colors[i], label=labels[i])

        plt.show()

    def getLoopWaypoints(self):
        lw = counterClockwiseRect(self.waypoints, self.boat, buoy_offset=20)
        return lw


class MockBoat:

    def __init__(self, position):
        self.position = position

    def getPosition(self):
        return self.position
