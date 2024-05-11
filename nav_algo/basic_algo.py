import nav_algo.boat as b
from nav_algo.event_helper.navigation_helper import newSailingAngle 
import numpy as np
from nav_algo.navigation_utilities import getServoAnglesImpl
import math

class BasicAlgo:
    """
    The old navigation algorithm - this currently sucks and needs a major
    refactor if it's ever going to be used. Testing if branch updates.
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
        return wind_direction % 360
        # return (wind_direction - heading_direction) % 360
    
    def setSail(windDir, currHead):
        """
        Calculate and set the new sail angle given ... Figure out what parameters are needed
        """
        cWindDir = BasicAlgo.calibrate_wind_direction(windDir, currHead)
        # wind is blowing in the same direction as the sailing direction (run), this range
        # sets a 20 degree buffer zone so that the sail does not always flip.
        if 0 <= cWindDir < 10 or 350 < cWindDir < 360:
            return 90
        elif 210 < cWindDir <= 350:
            return round(((7/15)*cWindDir - 80)/5)*5
        elif 10 <= cWindDir < 150:
            return round(((7/15)*cWindDir - 88)/5)*5
        # no go zone (150 <= cWindDir <= 210)
        else:
            return 0

    def setRudder(currLoc, tacking, tackingPoint, headingDir, currDest):
        """
        Calculate and set the new rudder angle given ...
        """
        # calculate based on whether we're tacking or not. if yes, calculate 
        # direction to tacking point, if not, calcualte direction to dest. 
        if tacking: 
            final = tackingPoint
            x_distance = final[0] - currLoc.getX()
            y_distance = final[1] - currLoc.getY()
        else:
            x_distance = currDest.getX() - currLoc.getX()
            y_distance = currDest.getY() - currLoc.getY()
        # 'final' and 'currLoc' are sometimes 'Vector' or 'tuple' types. Bracket indexing works when they are tuples but not when they are vectors.
        targetBearing = np.arctan2(y_distance, x_distance) * 180 / np.pi
        # print("TB", targetBearing)
        diff = np.mod((targetBearing) - (headingDir) + 180, 360) - 180
        # print("diff", diff)
        rudderAngle = (diff / 180) * -30
        # print("rudder angle raw", rudderAngle)
        rudderAngle = np.floor(rudderAngle / 5) * 5
        return rudderAngle       
    
        # if x_distance > 0 and y_distance == 0:
        #     angle = 0
        # elif x_distance == 0 and y_distance > 0:
        #     angle = 90
        # elif x_distance < 0 and y_distance == 0:
        #     angle = 180
        # elif x_distance == 0 and y_distance < 0:
        #     angle = 270
        # elif x_distance > 0 and y_distance > 0:
        #     angle = np.rad2deg(np.arctan(y_distance/x_distance))
        # elif x_distance < 0  and y_distance > 0:
        #     angle = 180 + np.rad2deg(np.arctan(y_distance/x_distance))
        # elif x_distance < 0  and y_distance < 0:
        #     angle = 180 + np.rad2deg(np.arctan(y_distance/x_distance))
        # elif x_distance > 0  and y_distance < 0:
        #     angle = 360 + np.rad2deg(np.arctan(y_distance/x_distance))
        # else:
        #     print("already at final destination")
        
        # final_angle = angle - headingDir
        # if final_angle > 0 and final_angle <= 180:
        #     #turn counter-clockwise
        #     return -round((.05 * (final_angle)))*5
        # elif final_angle < 360 and final_angle > 180:
        #     #turn clockwise
        #     final_angle = 360 - final_angle
        #     return round(.05 * (final_angle))*5
        # elif final_angle < 0 and final_angle >= -180:
        #     #turn clockwise
        #     return -round(.05 * (angle))*5
        # elif final_angle < -180 and final_angle >= -360:
        #     #turn counter-clockwise
        #     final_angle = 360 + final_angle
        #     return -round(.05 * (final_angle))*5
        # else:
        #     print ("invalid abs(final_angle) > 360")


    def inNoGo(headingDir, windDir):
        """
        Checks if the boat is currently in the no go zone (within )
        """
        calWind = BasicAlgo.calibrate_wind_direction(windDir, headingDir)
        if 150 < calWind < 210:
            return True
        else:
            return False
        
    def calculateDistToDest(currLoc, destLoc):
        """
        Calculates the euclidean distance to the destination
        """
        cx, cy = currLoc.getX(), currLoc.getY()
        dx, dy = destLoc.getX(), destLoc.getY()
        return math.sqrt((cx - dx)**2 + (cy - dy)**2)
    
    def calculateTP(currentLocation, destination, windDirection, headingDirection):
        """
        Calcualte tacking point to begin tacking. uses winddir + dest
        Assuming that the boat is heading towards the positive x-axis and the destination
        """
        x = currentLocation.getX()
        y = currentLocation.getY()
        dist2Dest = BasicAlgo.calculateDistToDest(currentLocation, destination)
        windWRThead = BasicAlgo.calibrate_wind_direction(windDirection, headingDirection) # does this line mess with calculations?
        if windWRThead >= 180 and windWRThead <= 210:
            x_TP = x + dist2Dest*np.cos(np.deg2rad(45-windWRThead))*np.sin(np.deg2rad(45+windWRThead))
            y_TP = y - dist2Dest*np.cos(np.deg2rad(45-windWRThead))*np.cos(np.deg2rad(45+windWRThead))
            tackingPoint = (x_TP, y_TP)
        elif windWRThead >= 150 and windWRThead <= 180:
            windWRThead = 360 - windWRThead
            x_TP = x + dist2Dest*np.cos(np.deg2rad(45-windWRThead))*np.sin(np.deg2rad(45+windWRThead))
            y_TP = y + dist2Dest*np.cos(np.deg2rad(45-windWRThead))*np.cos(np.deg2rad(45+windWRThead))
            tackingPoint = (x_TP, y_TP)
        return tackingPoint

    def step(self, currentLoc, destination, tacking, tpoint, tduration, headingDir, windDir):
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
                tpoint = BasicAlgo.calculateTP(currentLoc, destination, windDir, headingDir)
        return BasicAlgo.setSail(windDir, headingDir), BasicAlgo.setRudder(currentLoc, tacking, tpoint, headingDir, destination), tacking, tpoint, tduration
            
