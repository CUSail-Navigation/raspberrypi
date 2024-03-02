import nav_algo.boat as b
import numpy as np
from nav_algo.navigation_helper import *
from nav_algo.navigation_utilities import getServoAnglesImpl
    
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
    
    def updateSensors(self, boat : b.BoatController, waypoint):
        """
        Updates the fields given by the sensor outputs. (Update after knowing 
        what angle type). Figure out what parameters 
        """ 
        self.headingDir = 0 # set 
        self.windDir = 0 # set
        self.currLoc = None # set
    
    def setSail(self):
        """
        Calculate and set the new sail angle given ... Figure out what parameters are needed
        """
        if 30 < self.windDir < 180:
            # self.servos.setSail(round((7/15)*self.windDir + 186)*5)
            return round((7/15)*self.windDir + 186)*5
        elif 180 < self.windDir < 330:
            # self.servos.setSail(round((7/15)*self.windDir + 6)*5)
            return round((7/15)*self.windDir + 6)*5

    def setRudder(self):
        """
        Calculte and set the new rudder angle given ...
        """
        # calculate based on whether we're tacking or not. if yes, calculate 
        # direction to tacking point, if not, calcualte direction to dest. 
        if self.tacking:
            # self.servos.setTail(round(.25 * (self.tackingPoint-self.headingDir))*5)
            return round(.25 * (self.tackingPoint-self.headingDir))*5
        else:
            # self.servos.setTail(round(.25 * (self.currDest-self.headingDir))*5)
            return round(.25 * (self.currDest-self.headingDir))*5

    def inNoGo(self):
        """
        Checks if the boat is currently in the no go zone (within )
        """
        if abs(self.headingDir - self.windDir) < 30 or abs(self.headingDir - self.windDir) > 330:
            return True
        else:
            return False
    
    def calcualteTP(self, currentLocation):
        """
        Calcualte tacking point to begin tacking. uses winddir + dest
        Assuming that the boat is heading towards the positive x-axis and the destination
        """
        x = self.currentLoc[0]
        y = self.currentLoc[1]
        windDir = self.windDir % 360
        if windDir >= 0 and windDir <= 30:
            x_TP = x + self.distToDest*np.cos(np.deg2rad(45-windDir))*np.sin(np.deg2rad(45+windDir))
            y_TP = y - self.distToDest*np.cos(np.deg2rad(45-windDir))*np.cos(np.deg2rad(45+windDir))
            self.tackingPoint = (x_TP, y_TP)
        elif windDir >= 330 and windDir <= 359:
            windDir = 360 - windDir
            x_TP = x + self.distToDest*np.cos(np.deg2rad(45-windDir))*np.sin(np.deg2rad(45+windDir))
            y_TP = y + self.distToDest*np.cos(np.deg2rad(45-windDir))*np.cos(np.deg2rad(45+windDir))
            self.tackingPoint = (x_TP, y_TP)
        return self.tackingPoint

    def step(self, currentLoc, destination):
        """
        Sail. Todo: implement time buffer for tacking. if tacking for > 2 mins?
        If we call this function every 4 seconds --> tackingDur > 30
        """
        self.updateSensors()
        self.currDest = destination
        # check if current location is within range of destination with euclidian norm
        if self.tacking:
            if not self.inNoGo():
                if self.tackingDuration > 30:
                    # only get out of tacking after 2 mins
                    self.tacking = False
                    self.tackingDuration = 0
                else:
                    self.tackingDuration += 1
            else:
                self.tackingDuration += 1   
        else:
            if self.inNoGo():
                self.tacking = True
                self.tackingDuration = 0
                self.tackingPoint = self.calculateTP(currentLoc)
        return self.setSail(), self.setRudder()
            


        






    # Old step func
    # def step(self, boat : b.BoatController, waypoint):
    #     intended_angle = newSailingAngle(boat, waypoint)

    #     abs_wind_dir = boat.sensors.wind_direction # TODO is this right?
    #     yaw = boat.sensors.yaw
    #     sail, rudder = getServoAnglesImpl(abs_wind_dir, yaw, intended_angle)
    #     return sail, rudder
