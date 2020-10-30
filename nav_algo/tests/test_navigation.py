import unittest
import numpy as np
import nav_algo.navigation as nav
import nav_algo.coordinates as coord
import nav_algo.navigation_helper as help


class TestNavigationMethods(unittest.TestCase):
    def setUp(self):
        self.waypoints = [(42.444241, 76.481933), (42.446016, 76.484713)]
        self.nav_controller = nav.NavigationController(self.waypoints,
                                                       simulation=True)

        # mock sensor readings - some of these aren't always used
        self.nav_controller.boat.sensors.wind_direction = 45.0
        self.nav_controller.boat.sensors.fix = True
        self.nav_controller.boat_position = coord.Vector(x=5.0, y=0.0)
        self.nav_controller.boat.sensors.velocity = coord.Vector(x=0.3, y=0.4)
        self.nav_controller.boat_to_target = self.nav_controller.current_waypoint.vectorSubtract(
            self.nav_controller.boat_position)

    def test_polar(self):
        # directly upwind
        v = self.nav_controller.polar(0.0)
        self.assertAlmostEqual(v.x, 0.0)
        self.assertAlmostEqual(v.y, 0.0)

        # directly downwind
        v = self.nav_controller.polar(180.0)
        self.assertAlmostEqual(v.x, 0.0)
        self.assertAlmostEqual(v.y, 0.0)

        # port
        v = self.nav_controller.polar(270.0)
        self.assertAlmostEqual(v.magnitude(), 1.0)
        self.assertAlmostEqual(
            v.angle(), 270.0 + self.nav_controller.boat.sensors.wind_direction)

        # starboard
        v = self.nav_controller.polar(30.0)
        self.assertAlmostEqual(v.magnitude(), 1.0)
        self.assertAlmostEqual(
            v.angle(), 30.0 + self.nav_controller.boat.sensors.wind_direction)

    def test_optAngle(self):
        # best angle is directly to target on the left side
        self.nav_controller.boat_to_target = coord.Vector(x=1.0, y=0.0)
        angle, vmg = self.nav_controller.optAngle(False)  # left side
        self.assertAlmostEqual(angle, 0.0)
        self.assertAlmostEqual(vmg, 1.0)
        angle, vmg = self.nav_controller.optAngle(True)  # right side
        self.assertAlmostEqual(angle, 66.0)
        self.assertAlmostEqual(vmg, np.cos(coord.degToRad(66.0)))

        # best angle is directly to target on the right side
        self.nav_controller.boat_to_target = coord.Vector(x=-1.0, y=0.0)
        angle, vmg = self.nav_controller.optAngle(True)  # right side
        self.assertAlmostEqual(angle, 180.0)
        self.assertAlmostEqual(vmg, 1.0)
        angle, vmg = self.nav_controller.optAngle(False)  # left side
        self.assertAlmostEqual(angle, 246.0)
        self.assertAlmostEqual(vmg, -1.0 * np.cos(coord.degToRad(246.0)))

        # target is directly upwind (get as close as possible)
        self.nav_controller.boat_to_target = coord.Vector(x=1.0, y=1.0)
        angle, vmg = self.nav_controller.optAngle(True)  # right side
        self.assertAlmostEqual(angle, 66.0)
        self.assertAlmostEqual(
            vmg,
            np.sin(coord.degToRad(66.0)) * np.sin(coord.degToRad(45.0)) +
            np.cos(coord.degToRad(66.0)) * np.cos(coord.degToRad(45.0)))
        angle, vmg = self.nav_controller.optAngle(False)  # left side
        self.assertAlmostEqual(angle, 24.0)
        self.assertAlmostEqual(
            vmg,
            np.sin(coord.degToRad(24.0)) * np.sin(coord.degToRad(45.0)) +
            np.cos(coord.degToRad(24.0)) * np.cos(coord.degToRad(45.0)))

        # target is directly downwind (get as close as possible)
        self.nav_controller.boat_to_target = coord.Vector(x=-1.0, y=-1.0)
        angle, vmg = self.nav_controller.optAngle(True)  # right side
        self.assertAlmostEqual(angle, 204.0)
        self.assertAlmostEqual(
            vmg,
            np.sin(coord.degToRad(204.0)) * np.sin(coord.degToRad(225.0)) +
            np.cos(coord.degToRad(204.0)) * np.cos(coord.degToRad(225.0)))
        angle, vmg = self.nav_controller.optAngle(False)  # left side
        self.assertAlmostEqual(angle, 246.0)
        self.assertAlmostEqual(
            vmg,
            np.sin(coord.degToRad(246.0)) * np.sin(coord.degToRad(225.0)) +
            np.cos(coord.degToRad(246.0)) * np.cos(coord.degToRad(225.0)))

    def test_newSailingAngle(self):
        # should turn back toward origin on the right
        angle = self.nav_controller.newSailingAngle()
        self.assertAlmostEqual(
            angle,
            self.nav_controller.boat_position.inverse().angle())

        self.nav_controller.boat_position = coord.Vector(x=-50.0, y=0.0)
        self.nav_controller.boat_to_target = self.nav_controller.current_waypoint.vectorSubtract(
            self.nav_controller.boat_position)
        angle = self.nav_controller.newSailingAngle()
        self.assertAlmostEqual(angle, 0.0)

        # beating
        self.nav_controller.boat_position = coord.Vector(x=-50.0, y=-50.0)
        self.nav_controller.boat_to_target = self.nav_controller.current_waypoint.vectorSubtract(
            self.nav_controller.boat_position)
        angle = self.nav_controller.newSailingAngle()
        self.assertAlmostEqual(angle, 66.0)  # right side is closer to heading

        self.nav_controller.boat_position = coord.Vector(x=-50.0, y=-50.0)
        self.nav_controller.boat.sensors.velocity = coord.Vector(x=0.4, y=0.3)
        self.nav_controller.boat_to_target = self.nav_controller.current_waypoint.vectorSubtract(
            self.nav_controller.boat_position)
        angle = self.nav_controller.newSailingAngle()
        self.assertAlmostEqual(angle, 24.0)  # left side is closer to heading

        # tacking - heading is closer to right, but target is closer to left
        self.nav_controller.boat_position = coord.Vector(x=-20.0, y=-5.0)
        self.nav_controller.boat.sensors.velocity = coord.Vector(x=0.3, y=0.4)
        self.nav_controller.boat_to_target = self.nav_controller.current_waypoint.vectorSubtract(
            self.nav_controller.boat_position)
        angle = self.nav_controller.newSailingAngle()
        self.assertAlmostEqual(angle, 14.0)

        # tacking - heading is closer to left, but target is closer to right
        self.nav_controller.boat_position = coord.Vector(x=-5.0, y=-20.0)
        self.nav_controller.boat.sensors.velocity = coord.Vector(x=0.4, y=0.3)
        self.nav_controller.boat_to_target = self.nav_controller.current_waypoint.vectorSubtract(
            self.nav_controller.boat_position)
        angle = self.nav_controller.newSailingAngle()
        self.assertAlmostEqual(angle, 76.0)

    def test_station_keeping(self):
        # entry- given 4 square waypoints, return: 1) entry point 2) center
        # corner waypoint order: NW, NE, SE, SW        
        waypoints = [(5, 45), (45, 45), (5, 45), (5, 5)]
        # West entry
        self.nav_controller.boat_position = coord.Vector(x=0, y=25)
        new_waypoints = self.nav_controller.station_keeping(
            waypoints, 10, "ENTRY")
        expected_waypoints = [(5, 25), (25, 25)]
        self.assertAlmostEqual(new_waypoints, expected_waypoints)
        # North entry
        self.nav_controller.boat_position = coord.Vector(x=25, y=55)
        new_waypoints = self.nav_controller.station_keeping(
            waypoints, 10, "ENTRY")
        expected_waypoints = [(25, 45), (25, 25)]
        self.assertAlmostEqual(new_waypoints, expected_waypoints)
        # East entry
        self.nav_controller.boat_position = coord.Vector(x=55, y=25)
        new_waypoints = self.nav_controller.station_keeping(
            waypoints, 10, "ENTRY")
        expected_waypoints = [(45, 25), (25, 25)]
        self.assertAlmostEqual(new_waypoints, expected_waypoints)
        # South entry
        self.nav_controller.boat_position = coord.Vector(x=25, y=0)
        new_waypoints = self.nav_controller.station_keeping(
            waypoints, 10, "ENTRY")
        expected_waypoints = [(25, 5), (25, 25)]
        self.assertAlmostEqual(new_waypoints, expected_waypoints)


        #keep- given position, yaw, and wind, return: 1) Four 90 degree waypoints to loop
        #scenario 1- (facing east, optimal angle is 45)
        self.nav_controller.boat_position = coord.Vector(x=0.0, y=0.0)
        self.nav_controller.boat.sensors.yaw = 0.0
        
        #ccw path
        self.nav_controller.boat.sensors.wind_direction = 45.0
        new_waypoints = self.nav_controller.station_keeping(
            [], 10, "KEEP")
         expected_waypoints=[(5*math.sqrt(2),5*math.sqrt(2)),
            (-5*math.sqrt(2),5*math.sqrt(2)),
            (-5*math.sqrt(2),-5*math.sqrt(2)),
            (5*math.sqrt(2),-5*math.sqrt(2))]
        self.assertAlmostEqual(new_waypoints, expected_waypoints)   

        #cw path
        self.nav_controller.boat.sensors.wind_direction = 315
        new_waypoints = self.nav_controller.station_keeping(
            [], 10, "KEEP")        
        first_angle=math.radians(45)
         expected_waypoints=[(5*math.sqrt(2),-5*math.sqrt(2)),
            (-5*math.sqrt(2),-5*math.sqrt(2)),
            (-5*math.sqrt(2),5*math.sqrt(2)),
            (5*math.sqrt(2),5*math.sqrt(2))]
     
        self.assertAlmostEqual(new_waypoints, expected_waypoints)


        # scenario 2- not facing east(60), non-45 optimal angle (15)
        self.nav_controller.boat_position = coord.Vector(x=5.0, y=5.0)
        self.nav_controller.boat.sensors.yaw = 60.0

        
        #ccw path
        self.nav_controller.boat.sensors.wind_direction = 45.0
        new_waypoints = self.nav_controller.station_keeping(
            [], 10, "KEEP")
        first_angle=math.radians(60+45)

        expected_waypoints=[(5+10*math.cos(first_angle),5+10*math.sin(first_angle)),
            (5+10*math.cos(first_angle+math.pi/2),5+10*math.sin(first_angle+math.pi/2)),
            (5+10*math.cos(first_angle+math.pi),5+10*math.sin(first_angle+math.pi)),
            (5+10*math.cos(first_angle+3*math.pi/2),5+10*math.sin(first_angle+3*math.pi/2))]

        self.assertAlmostEqual(new_waypoints, expected_waypoints)

        #cw path
        self.nav_controller.boat.sensors.wind_direction = 90.0
        new_waypoints = self.nav_controller.station_keeping(
            [], 10, "KEEP")        
        first_angle=math.radians(60-15)

        expected_waypoints=[(10*math.cos(first_angle),10*math.sin(first_angle)),
            (10*math.cos(first_angle-math.pi/2),10*math.sin(first_angle-math.pi/2)),
            (10*math.cos(first_angle-math.pi),10*math.sin(first_angle-math.pi)),
            (10*math.cos(first_angle-3*math.pi/2),10*math.sin(first_angle-3*math.pi/2))]
        self.assertAlmostEqual(new_waypoints, expected_waypoints)

        #exit- 4 waypoints, get closest exit
        waypoints = [(5, 45), (45, 45), (5, 45), (5, 5)]    #center=(25,25)
        #West exit
        expected_exit=(-5,25)
        self.nav_controller.boat_position = coord.Vector(x=15, y=25)
        exit_waypoint = self.nav_controller.station_keeping(
            waypoints, 10, "EXIT")    
        self.assertAlmostEqual(exit_waypoint, expected_exit)

        #North exit
        expected_exit=(25,55)
        self.nav_controller.boat_position = coord.Vector(x=25, y=35)
        exit_waypoint = self.nav_controller.station_keeping(
            waypoints, 10, "EXIT")    
        self.assertAlmostEqual(exit_waypoint, expected_exit)

        #East exit
        expected_exit=(55,25)
        self.nav_controller.boat_position = coord.Vector(x=35, y=25)
        exit_waypoint = self.nav_controller.station_keeping(
            waypoints, 10, "EXIT")    
        self.assertAlmostEqual(exit_waypoint, expected_exit)        


        #South exit
        expected_exit=(25,-5)
        self.nav_controller.boat_position = coord.Vector(x=25, y=15)
        exit_waypoint = self.nav_controller.station_keeping(
            waypoints, 10, "EXIT")    
        self.assertAlmostEqual(exit_waypoint, expected_exit)



            
        
if __name__ == '__main__':
    unittest.main()
