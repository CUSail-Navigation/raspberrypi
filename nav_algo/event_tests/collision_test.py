import matplotlib.pyplot as plt
import sys

sys.path.append("..")
from nav_algo.event_helper.collision_event.collision_avoidance import *
import nav_algo.coordinates as coord

class CollisionTest:

    def __init__(self):
        print("Starting test for Collision.")

        self.waypoints = [(0,0), (0,10), (100, 5)]

        plt.figure()
        plt.plot(self.waypoints[0][0], self.waypoints[0][1], 'o', color="orange")
        plt.plot(self.waypoints[1][0], self.waypoints[1][1], 'o', color="orange")
        plt.plot(self.waypoints[2][0], self.waypoints[2][1], 'o', color="orange")


        self.waypoints = [
            coord.Vector(x = w[0], y = w[1]) for w in self.waypoints
        ]
        collisionwaypoints = collisionAvoidance(self.waypoints)

        colors = ['lime', 'aqua', 'indigo', 'magenta']
        labels = ['first', 'second', 'third', 'fourth']
        for i in range(len(collisionwaypoints)):
            plt.plot(collisionwaypoints[i].x, collisionwaypoints[i].y, 'x', color=colors[i], label=labels[i])
        plt.show()

if __name__ == "__main__":
    CollisionTest()