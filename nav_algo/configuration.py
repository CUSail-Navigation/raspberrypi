import json
import nav_algo.sensors as sens
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
from nav_algo.events import Events
from nav_algo.rl_algo import RL
from nav_algo.basic_algo import BasicAlgo

class NavigationConfiguration:

    def __init__(self, config_filename, waypoint_filename):
        obj = None
        with open(config_filename) as f:
            obj = json.load(f)
        
        mock_gps = obj["peripherals"]["gps"] == "fake"
        mock_imu = obj["peripherals"]["imu"] == "fake"
        mock_anemometer = obj["peripherals"]["anemometer"] == "fake"
        
        waypoints = self.readWaypoints(waypoint_filename)
        coord_sys = coord.CoordinateSystem(waypoints[0][0], waypoints[0][1])

        sensor_data = sens.sensorData(coordinate_system=coord_sys,
                                      mock_gps=mock_gps,
                                      mock_imu=mock_imu,
                                      mock_anemometer=mock_anemometer)
        
        mock_servos = obj["peripherals"]["servos"] == "fake"
        boat_data = boat.BoatController(coord_sys, sensor_data, mock_servos)

        algo = None
        if obj["algo"]["type"] == "rl":
            algo = RL(obj["algo"]["model_path"])
        else:
            algo = BasicAlgo()

        #TODO do something with all of this and decide output method
    
    def readWaypoints(filename):
        waypoints = []
        with open(filename, 'r') as f:
            for line in f:
                line = line.replace('\n',"")
                
                l = line.split(",")
                waypoints.append((float(l[0]), float(l[1])))
        return waypoints

