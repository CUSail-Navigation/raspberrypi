import matplotlib.pyplot as plt
import sys

sys.path.append("..")
from navigation_helper import *
from navigation_utilities import *


class PrecisionTest:

    def __init__(self):
        print("Starting test for precision navigation.")

        self.waypoints = [(205, 93.5), (205, 96.5), (248, 70), (248, 120)]
        # self.waypoints = generateBuoys()

        plt.figure()
        xs = [w[0] for w in self.waypoints]
        ys = [w[1] for w in self.waypoints]
        plt.plot(xs, ys, 'o', color="orange")

        # gen_waypoints = precisionNavigationImpl(self.waypoints)
        gen_waypoints = precisionNavigationImpl(self.waypoints)
        xs = [w[0] for w in gen_waypoints]
        ys = [w[1] for w in gen_waypoints]
        colors = ['r', 'y', 'g', 'b', 'm']
        for i in range(len(xs)):
            plt.plot(xs[i], ys[i], 'x')

        plt.show()