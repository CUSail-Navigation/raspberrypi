import matplotlib.pyplot as plt
#import sys
import sys
from pathlib import Path

# Append the parent directory of `nav_algo` to sys.path
parent_dir = str(Path(__file__).resolve().parent.parent)
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

print("Parent dir:",parent_dir)
for path in sys.path:
    print(path)

#sys.path.append("..")
from nav_algo.navigation_utilities import *
#from navigation_helper import *
from event_helper.search import search_helper


class SearchTest:

    def __init__(self):
        print("Starting test for search.")

        self.waypoints = [(0, 0)]

        plt.figure()
        xs = [w[0] for w in self.waypoints]
        ys = [w[1] for w in self.waypoints]
        plt.plot(xs, ys, 'o', color="red")

        gen_waypoints = search_helper(self.waypoints)
        xs = [w[0] for w in gen_waypoints]
        ys = [w[1] for w in gen_waypoints]
        xss = np.array(xs)
        yss = np.array(ys)
        for i in range(len(xs)):
            plt.plot(xs[i], ys[i], '-x', color='black', linewidth=2)
            plt.quiver(xss[:-1], yss[:-1], xss[1:]-xss[:-1], yss[1:]-yss[:-1],
                       scale_units='xy', angles='xy', scale=1, color='black', width=0.005)

        plt.show()
