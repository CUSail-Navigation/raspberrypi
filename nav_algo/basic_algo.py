import nav_algo.boat as b
import numpy as np
from nav_algo.navigation_helper import *
from nav_algo.navigation_utilities import getServoAnglesImpl
import math

class BasicAlgo:
    """
    The old navigation algorithm - this currently sucks and needs a major
    refactor if it's ever going to be used.
    """
    def __init__(self):
        # self.rudderAngle = 0 covered in servos
        # self.sailAngle = 0 covered in servos
        self.headingDir = 0
        self.windDir = 0
        self.distToDest = 0
        self.tacking = False
        self.tackingPoint = None
        self.tackingDuration = 0
        self.currLoc = None
        self.currDest = None 
    
    # def updateSensors(self, boat : b.BoatController, waypoint):
    #     """
    #     Updates the fields given by the sensor outputs. (Update after knowing 
    #     what angle type). Figure out what parameters 
    #     """ 
    #     self.headingDir = 0 # set 
    #     self.windDir = 0 # set
    #     self.currLoc = None # set
    def normalize_angle(angle):
        return angle % 360
    
    def calibrate_wind_direction(wind_direction, heading_direction):
        return (wind_direction - heading_direction) % 360
    
    def setSail(windDir, currHead):
        """
        Calculate and set the new sail angle given ... Figure out what parameters are needed
        """
        cWindDir = BasicAlgo.calibrate_wind_direction(windDir, currHead)
        if 30 < cWindDir < 180:
            # self.servos.setSail(round((7/15)*self.windDir + 186)*5)
            return round((7/15)*cWindDir + 186)*5
        elif 180 < cWindDir < 330:
            # self.servos.setSail(round((7/15)*self.windDir + 6)*5)
            return round((7/15)*cWindDir + 6)*5

    def setRudder(currLoc, tacking, tackingPoint, headingDir, currDest):
        """
        Calculate and set the new rudder angle given ...
        """
        # calculate based on whether we're tacking or not. if yes, calculate 
        # direction to tacking point, if not, calcualte direction to dest. 
        if tacking: 
            final = tackingPoint
        else:
            final = currDest
        
        x_distance = final[0] - currLoc[0]
        y_distance = final[1] - currLoc[1]
        if x_distance > 0 and y_distance == 0:
            angle = 0
        elif x_distance == 0 and y_distance > 0:
            angle = 90
        elif x_distance < 0 and y_distance == 0:
            angle = 180
        elif x_distance == 0 and y_distance < 0:
            angle = 270
        elif x_distance > 0 and y_distance > 0:
            angle = np.rad2deg(np.arctan(y_distance/x_distance))
        elif x_distance < 0  and y_distance > 0:
            angle = 180 + np.rad2deg(np.arctan(y_distance/x_distance))
        elif x_distance < 0  and y_distance < 0:
            angle = 180 + np.rad2deg(np.arctan(y_distance/x_distance))
        elif x_distance > 0  and y_distance < 0:
            angle = 360 + np.rad2deg(np.arctan(y_distance/x_distance))
        else:
            print("already at final destination")
        
        final_angle = angle - headingDir
        if final_angle > 0 and final_angle <= 180:
            #turn counter-clockwise
            return -round((.05 * (final_angle)))*5
        elif final_angle > 0 and final_angle > 180:
            #turn clockwise
            final_angle = 360 - final_angle
            return round(.05 * (final_angle))*5
        elif final_angle < 0 and final_angle >= -180:
            #turn clockwise
            return -round(.05 * (angle))*5
        elif final_angle < 0 and final_angle >= -360:
            #turn counter-clockwise
            final_angle = 360 + final_angle
            return -round(.05 * (final_angle))*5
        else:
            print ("invalid abs(final_angle) > 360")


    def inNoGo(headingDir, windDir):
        """
        Checks if the boat is currently in the no go zone (within )
        """
        if abs(headingDir - windDir) < 30 or abs(headingDir - windDir) > 330:
            return True
        else:
            return False
        
    def calculateDistToDest(currLoc, destLoc):
        """
        Calculates the euclidean distance to the destination
        """
        cx, cy = currLoc[0], currLoc[1]
        dx, dy = destLoc[0], destLoc[1]
        return math.sqrt((cx - dx)**2 + (cy - dy)**2)
    
    def calcualteTP(currentLocation, destination, windDirection):
        """
        Calcualte tacking point to begin tacking. uses winddir + dest
        Assuming that the boat is heading towards the positive x-axis and the destination
        """
        x = currentLocation[0]
        y = currentLocation[1]
        dist2Dest = BasicAlgo.calculateDistToDest(currentLocation, destination)
        windDir = windDirection % 360
        if windDir >= 0 and windDir <= 30:
            x_TP = x + dist2Dest*np.cos(np.deg2rad(45-windDir))*np.sin(np.deg2rad(45+windDir))
            y_TP = y - dist2Dest*np.cos(np.deg2rad(45-windDir))*np.cos(np.deg2rad(45+windDir))
            tackingPoint = (x_TP, y_TP)
        elif windDir >= 330 and windDir <= 359:
            windDir = 360 - windDir
            x_TP = x + dist2Dest*np.cos(np.deg2rad(45-windDir))*np.sin(np.deg2rad(45+windDir))
            y_TP = y + dist2Dest*np.cos(np.deg2rad(45-windDir))*np.cos(np.deg2rad(45+windDir))
            tackingPoint = (x_TP, y_TP)
        return tackingPoint

    def step(currentLoc, destination, tacking, tpoint, tduration, headingDir, windDir):
        """
        Sail. Todo: implement time buffer for tacking. if tacking for > 2 mins?
        If we call this function every 4 seconds --> tackingDur > 30
        """
        # self.updateSensors()
        currDest = destination
        # check if current location is within range of destination with euclidian norm
        if tacking:
            #  Finding 
            if not BasicAlgo.inNoGo(headingDir, windDir):
                if tduration > 30:
                    # only get out of tacking after 2 mins
                    tacking = False
                    tduration = 0
                else:
                    tduration += 1
            else:
                tduration += 1   
        else:
            if BasicAlgo.inNoGo(headingDir, windDir):
                tacking = True
                tduration = 0
                tpoint = BasicAlgo.calculateTP(currentLoc)
        return BasicAlgo.setSail(windDir, headingDir), BasicAlgo.setRudder(tacking, tpoint, headingDir, destination), tacking, tpoint, tduration
            


        






    # Old step func
    # def step(self, boat : b.BoatController, waypoint):
    #     intended_angle = newSailingAngle(boat, waypoint)

    #     abs_wind_dir = boat.sensors.wind_direction # TODO is this right?
    #     yaw = boat.sensors.yaw
    #     sail, rudder = getServoAnglesImpl(abs_wind_dir, yaw, intended_angle)
    #     return sail, rudder
