import matplotlib.pyplot as plt
import sys

sys.path.append("..")
from navigation_utilities import *
from navigation_helper import *


class SearchTest:

    def __init__(self):
        print("Starting test for search.")

        self.waypoints = [(0, 0)]

        plt.figure()
        xs = [w[0] for w in self.waypoints]
        ys = [w[1] for w in self.waypoints]
        plt.plot(xs, ys, 'o', color="red")

        gen_waypoints = search(self.waypoints)
        xs = [w[0] for w in gen_waypoints]
        ys = [w[1] for w in gen_waypoints]
        xss = np.array(xs)
        yss = np.array(ys)
        for i in range(len(xs)):
            plt.plot(xs[i], ys[i], '-x', color='black', linewidth=2)
            plt.quiver(xss[:-1], yss[:-1], xss[1:]-xss[:-1], yss[1:]-yss[:-1],
                       scale_units='xy', angles='xy', scale=1, color='black', width=0.005)

        plt.show()
