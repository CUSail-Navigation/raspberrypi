import numpy as np
import math
import random

def findAnglePrecisionNav(bouys):
    """Finds the angle of the orientation of the bouys.

        Args:
            buoys (list of (float, float)): The ordered buoy locations (top_l, top_r, bot_l, bot_r).

        Returns:
            (float): The angle of inclination of the bouys from the x-axis.

    """ 
    topMid = [((bouys[0][0]+bouys[1][0])/2), ((bouys[0][1] + bouys[1][1])/2)]
    bottomMid = [((bouys[2][0]+bouys[3][0])/2), ((bouys[2][1] + bouys[3][1])/2)]
    slopeY = topMid[1] - bottomMid[1]
    slopeX = topMid[0]-bottomMid[0]

    angle = np.arctan2(slopeY,slopeX)
    angle -= np.pi / 2.0 # in the buoy frame, 0 is up, not x-axis
    return angle

def teardrop_shaped_curve(start_pos, tval, height, width, angle=0):
    """Outputs a waypoint location according to the teardrop shaped function.

        Args:
            start_pos (float): coordinate of starting bouy position
            tval (float): radians for where the waypoint falls in the graph
            angle (float): radians for the angle of inclination of the orientation of the bouys

        Returns:
            (float,float): x and y coordinates of a waypoint
    """

    m = 3
    y = height * np.cos(tval)
    x = width * np.sin(tval) * (np.sin(0.5 * tval) ** m)

    return rotateAndTranslatePrecisionNav(x, y, angle, start_pos, height, width)

def rotateAndTranslatePrecisionNav(x, y, angle, start_pos, height, width):
    """Outputs the rotated x and y coordinates

        Args:
            x (float): x coordinate of waypoint
            y (float): y coordinate of waypoint
            angle (float): radians for the angle of inclination of the orientation of the bouys

        Returns:
            (float,float): x and y coordinates of rotated waypoint
    """
    # get the rotated start position on the parametric curve so we know how to shift
    orig_x = 0
    orig_y = height
    rot_x = orig_x*math.cos(angle)-orig_y*math.sin(angle) 
    rot_y = orig_x*math.sin(angle)+orig_y*math.cos(angle)

    newX = x*math.cos(angle)-y*math.sin(angle) + (start_pos[0] - rot_x)
    newY = x*math.sin(angle)+y*math.cos(angle) + (start_pos[1] - rot_y)
    return (newX, newY)

def generate_t_vals(num_waypoints):
    """Outputs a list of t values. Finds equally spaced t values based on number of waypoints to
       generate the waypoints along the curve

        Args:
            num_waypoints (integer): number of waypoints needed from the graph

        Returns:
            [(float)]: list of floats representing the t values
    """
    return np.linspace(0, 2.0 * np.pi, num_waypoints).tolist()

def PrecisionNavigationAlgo(bouys, num_waypoints):
    """Generates navigation waypoints from the precision nav buoy locations.

        Args:
            buoys (list of (float, float)): The ordered buoy locations (top_l, top_r, bot_l, bot_r).

        Returns:
            (list of (float, float): The generated waypoint locations.

    """
    # buoys:[topleft_buoy, topright_buoy, botleft_buoy, botright_buoy]
    topleft_buoy = bouys[0]
    topright_buoy = bouys[1]
    botleft_buoy = bouys[2]
    botright_buoy = bouys[3]
    
    # Scale the teardrop curve to the correct size
    height = np.sqrt((topleft_buoy[0] - botleft_buoy[0]) ** 2 + 
                     (topleft_buoy[1] - botleft_buoy[1]) **2) / 1.8
    
    width = np.sqrt((botleft_buoy[0] - botright_buoy[0]) ** 2 + 
                    (botleft_buoy[1] - botright_buoy[1]) **2) + 5.0

    t_values = generate_t_vals(num_waypoints)

    # start/finish position between top buoys
    start_pos = midpoint(topleft_buoy, topright_buoy)

    out_waypoints = []
    for i in range(len(t_values)):
        out_waypoints.append(teardrop_shaped_curve(start_pos, t_values[i], height,
                                                   width, 
                                                   findAnglePrecisionNav(bouys)))

    return out_waypoints

def midpoint(u, v):
    """
    Finds the midpoint from two inputted points
    """
    return (u[0] + v[0]) / 2, (u[1] + v[1]) / 2

def generateBuoys():
    """
    Function created to generate buoys that follow the spacing requirements of the event
    Used for testing the algorithm not waypoint calculation
    """
    # angle = random.random()*(2*math.pi)*math.pi/180
    angle = 0
    absSin = abs(np.sin(angle))
    absCos = abs(np.cos(angle))
    absSin2 = abs(np.sin(angle+2.06))
    absCos2 = abs(np.cos(angle+2.06))
    absSin3 = abs(np.sin(angle-2.06+math.pi/2))
    absCos3 = abs(np.cos(angle-2.06+math.pi/2))
    
    if angle < math.pi/2:

        firstPointX = random.random()*(200)
        firstPointY = random.random()*200
        
        secondPointX = absCos*3+firstPointX
        secondPointY = absSin*3+firstPointY

        thirdPointX = firstPointX - absSin3*50 
        thirdPointY = firstPointY - absCos3*50

        fourthPointX = absCos*50 + thirdPointX
        fourthPointY = absSin*50 + thirdPointY

    elif angle > math.pi*3/2:
        firstPointX = random.random()*(200)
        firstPointY = random.random()*200
        
        secondPointX = absSin*3+firstPointX
        secondPointY = firstPointY - absCos*3

        thirdPointX = firstPointX - absSin2*50
        thirdPointY = firstPointY - absCos2*50

        fourthPointX = absCos*50 + thirdPointX
        fourthPointY = thirdPointY - absSin*50
    elif angle < math.pi*3/2 and angle > math.pi:

        firstPointX = random.random()*(200)
        firstPointY = random.random()*200
        
        secondPointX = firstPointX - absSin*3
        secondPointY = firstPointY - absCos*3

        thirdPointX = firstPointX - absSin2*50
        thirdPointY = firstPointY + absCos2*50

        fourthPointX = thirdPointX - absCos*50 
        fourthPointY = thirdPointY - absSin*50
    else:

        firstPointX = random.random()*(200)
        firstPointY = random.random()*200
        
        secondPointX = firstPointX - absSin*3
        secondPointY = firstPointY + absCos*3

        thirdPointX = firstPointX + absSin2*50
        thirdPointY = firstPointY + absCos2*50

        fourthPointX = thirdPointX - absCos*50 
        fourthPointY = thirdPointY + absSin*50

    

    first = (firstPointX, firstPointY)
    second = (secondPointX, secondPointY)
    third = (thirdPointX, thirdPointY)
    fourth = (fourthPointX, fourthPointY)

    
    return [first, second, third, fourth]