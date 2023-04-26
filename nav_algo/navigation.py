import time
import numpy as np

import nav_algo.configuration as conf
import nav_algo.boat as boat
import nav_algo.coordinates as coord
import nav_algo.radio as radio
from nav_algo.events import Events
from nav_algo.navigation_helper import *
# from nav_algo.camera import Camera


class NavigationController:
    """A controller class for the navigation algorithm.
    Args:
        configuration (NavigationConfiguration: The configuration of the current execution.
    Attributes:
        DETECTION_RADIUS (float): How close we need to get to a waypoint.
        coordinate_system (CoordinateSystem): The global coordinate system.
        waypoints (list of Vector): Position vectors of waypoints.
        boat (BoatController): A representation of the boat.
        radio (Radio): Prints navigation data to the base station.
        current_waypoint (Vector): The current target waypoint.
        boat_position (Vector): The current position of the boat.
        boat_to_target (Vector): The vector from the boat to the target position.
    """
    def __init__(self, configuration : conf.NavigationConfiguration):
        self.configuration = configuration
        self.DETECTION_RADIUS = 5.0

        self.configuration.write_output(
            "Using lat/long point ({}, {}) as the center of the coordinate system.\n"
            .format(self.waypoints[0].latitude, self.waypoints[0].longitude))
        self.configuration.write_output("Waiting for GPS fix...\n")

        # wait until we know where we are
        while self.boat.sensors.velocity is None:
            self.boat.sensors.readAll() # ok if this is blocking
        
        self.configuration.write_output(
            "Established GPS fix. Beginning navigation...\n")
        # self.current_waypoint = self.waypoints.pop(0)
        # self.current_waypoint = self.waypoints[desired_fst_waypoint]
        # TODO: add modified ^ to event algos before each navigate call

        # If the event is fleet race, we don't care about the algo, just set angles
        # NOTE commands should end with \n, send 'q' to quit, angles are space delineated 'main tail'
        if self.configuration.event == Events.FLEET_RACE:
            self.fleetRace()

        elif self.configuration.event == Events.ENDURANCE:
            self.endurance()

        elif self.configuration.event == Events.STATION_KEEPING:
            # TODO find an optimal radius, 10m for now
            buoy_waypoints = self.waypoints
            exit_before = 300
            circle_radius = 10
            self.waypoints = stationKeeping(buoy_waypoints,
                                            circle_radius,
                                            "ENTRY",
                                            boat=self.boat)
            self.current_waypoint = self.waypoints.pop(0)
            self.navigate()

            # Set timer
            start_time = time.time()
            loop_waypoints = stationKeeping(buoy_waypoints,
                                            circle_radius,
                                            "KEEP",
                                            boat=self.boat)
            while time.time() - start_time < exit_before:
                self.waypoints = loop_waypoints
                self.current_waypoint = self.waypoints.pop(0)
                self.navigate()
            self.waypoints = stationKeeping(buoy_waypoints,
                                            circle_radius,
                                            "EXIT",
                                            boat=self.boat)

        elif self.configuration.event == Events.PRECISION_NAVIGATION:
            self.waypoints = precisionNavigation(self.waypoints)
        elif self.configuration.event == Events.COLLISION_AVOIDANCE:
            self.waypoints = collisionAvoidance(self.waypoints)
            self.current_waypoint = self.waypoints[0]
            self.navigateDetection()
        elif self.configuration.event == Events.SEARCH:
            self.waypoints = search(self.waypoints, boat=self.boat)
            self.current_waypoint = self.waypoints[0]
            self.navigateDetection(event=Events.SEARCH)

        self.current_waypoint = self.waypoints.pop(0)
        self.navigate()

        # TODO Clean up ports
        self.radio.serialStream.close()

    def navigate(self):
        """ Execute the navigation algorithm.
        This is a blocking call that runs until all waypoints have been hit.
        """
        while self.current_waypoint is not None:
            # read for a quit signal ('q') or manual override ('o')
            try:
                self.radio.receiveString()
            except:
                pass

            while self.radio.fleetRace:
                # manual override has been engaged, wait for autopilot signal ('a')
                try:
                    self.radio.receiveString()
                except:
                    pass
                self.boat.updateSensors()
                self.radio.printData(self.boat)

            all_waypts = []
            all_waypts.append(self.current_waypoint)
            for pt in self.waypoints:
                all_waypts.append(pt)
            self.radio.printAllWaypoints(all_waypts)
            time.sleep(0.35)  # TODO how often should this run?
            
            self.boat.updateSensors()
            self.boat_position = self.boat.getPosition()
            self.radio.printData(self.boat)

            if self.boat_position.xyDist(
                    self.current_waypoint) < self.DETECTION_RADIUS:
                # hit waypoint -- send data back to basestation
                self.radio.printHitWaypoint(self.current_waypoint)

                if len(self.waypoints) > 0:
                    self.current_waypoint = self.waypoints.pop(0)
                else:
                    self.current_waypoint = None
                    break
                    
            try:
                print("hello")
                
                actor = Actor()
                actor.load_state_dict(torch.load('./nav_algo/best_actor_sailboat.pickle', map_location=torch.device('cpu')))
                actor = actor.to(torch.double)
                actor.eval()                
                '''
                '''
                # order of things 
                # 1. x velocity 
                vel_x = self.boat.sensors.velocity.x
                # 2. y velocity
                vel_y = self.boat.sensors.velocity.y
                # 3. angular velocity 
                vel_angular = self.boat.sensors.angular_velocity
                # 4. sail angle
                sail_angle = self.boat.servos.currentSail
                # 5. rudder angle 
                rudder_angle = self.boat.servos.currentTail
                # 6. relative wind x
                rel_wind_x = np.cos(self.boat.sensors.rawWind*np.pi/180)
                # 7. relative wind y 
                rel_wind_y = np.sin(self.boat.sensors.rawWind*np.pi/180)
                # 8. distance from goal x component 
                dist_goal_x = np.absolute(self.boat_position.x - self.current_waypoint.x)
                # 9. distance from goal y component
                dist_goal_y = np.absolute(self.boat_position.y - self.current_waypoint.y)


                state_vector = np.array([vel_x, vel_y, vel_angular, sail_angle, rudder_angle, rel_wind_x,
                rel_wind_y, dist_goal_x, dist_goal_y])
                
                
                tensor = torch.from_numpy(state_vector)
                print(tensor.double())
                sailing_angle = actor.get_action(tensor)
                print(sailing_angle) # np array mult by 90 for sail 30 for rudder 
                final_output = np.array([sailing_angle[0]*90, sailing_angle[1]*30])
                
                print("rel_wind_x and y : {}, {}".format(rel_wind_x, rel_wind_y))
                print("servo_angles : {}, {}".format(final_output[0], final_output[1]))
                self.boat.servos.setTail(final_output[1])
                self.boat.servos.setSail(final_output[0])
                
            except:
                raise Exception("Somethign went wrong")
            ### ========================================= ###
            # OLD ALGO CODE 
                final_output = newSailingAngle(self.boat, self.current_waypoint)
            ### ========================================= ###

                self.boat.setServos(final_output)

    def fleetRace(self):
        self.configuration.write_output(
            "Starting Fleet Race\nSend angles of the form 'sail_angle rudder_angle'\n"
        )
        while True:
            try:
                self.configuration.radio.receiveString()  # timeout is 1 sec
            except:
                pass
            self.configuration.boat.updateSensors()
            self.configuration.radio.printData()
    
    def endurance(self):
        # 7 hrs = 25200 sec
        exit_before = 25200
        start_time = time.time()
        loop_waypoints = counterClockwiseRect(self.configuration.waypoints,
                                              self.configuration.boat,
                                              buoy_offset=5)

        while (time.time() - start_time < exit_before):
            self.configuration.waypoints = loop_waypoints
            self.current_waypoint = self.configuration.waypoints.pop(0)
            self.navigate()
    
    def navigateDetection(self, event=Events.COLLISION_AVOIDANCE):
        # TODO: modify to implement collision avoidance
        while self.current_waypoint is not None:
            time.sleep(2)

            self.boat.updateSensors()
            self.boat_position = self.boat.getPosition()
            (buoy_coords, obst_coords) = Camera.read(self.boat.sensors.yaw,
                                                     self.boat_position.x,
                                                     self.boat_position.y)
            if (buoy_coords is not None & event == Events.SEARCH):
                # TODO: get buoy pos (buoy_waypoint)
                buoy_coords = coord.Vector(x=buoy_coords[0], y=buoy_coords[1])
                self.current_waypoint = buoy_coords
                self.waypoints = [buoy_coords]

            if (obst_coords is not None):
                obstacle_pos1 = obst_coords
                # TODO: get obstacle_pos at time t
                time.sleep(2)
                snd_read = Camera.read(self.boat.sensors.yaw,
                                       self.boat_position.x,
                                       self.boat_position.y)
                obstacle_pos2 = snd_read[0]
                avoidance_waypoint = assessCollision(obstacle_pos1,
                                                     obstacle_pos2, 2)
                if avoidance_waypoint is not None:
                    avoidance_waypoint = coord.Vector(x=avoidance_waypoint[0],
                                                      y=avoidance_waypoint[1])
                    self.current_waypoint = avoidance_waypoint
                    self.waypoints.insert(0, avoidance_waypoint)

            else:
                if self.boat_position.xyDist(
                        self.current_waypoint) < self.DETECTION_RADIUS:
                    if len(self.waypoints) > 1:
                        self.current_waypoint = self.waypoints[1]
                        del (self.waypoints[0])
                    else:
                        self.current_waypoint = None
                        del (self.waypoints[0])
                        break

            sailing_angle = newSailingAngle(self.boat, self.current_waypoint)
            self.boat.setServos(sailing_angle)
