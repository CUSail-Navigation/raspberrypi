import nav_algo.coordinates as coord
import random


class Engine():
    def __init__(self):
        pass

    def getVelocity(wind_vel, sail_angle, tail_angle, pitch, roll, yaw):
        # for now, return a random velocity vector
        return coord.Vector(x=random.randint(1, 5), y=random.randint(1, 5))
