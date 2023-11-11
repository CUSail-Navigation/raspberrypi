import numpy as np
"""
These functions are used by both the simulator and the nav algo.
The simulator cannot access the modules of the nav algo, so this
file should be self contained (i.e. not inherit from any other file).
Everything nav-algo-related that is needed by the simulator should 
exist within this file.
Within this file only, vectors are represented as (x, y) tuples.
"""

# New polar diagram
Polar_diagram = {
        0: 0,      # Directly into wind, speed is 0
        30: 2.5,  # 30 degrees off wind, speed is 2.5
        45: 4,    # 45 degrees off wind, speed is 4
        60: 4.5,  # 60 degrees off wind, speed is 4.5
        75: 4,    # 75 degrees off wind, speed is 4 
        90: 5,    # 90 degrees off wind, speed is 5
        110: 4.8, # 110 degrees off the wind, speed is 4.8 
        135: 4,   # 135 degrees off the wind, speed is 4 
        150: 3.5, # 150 degrees off the wind, speed is 3.5 
        180: 3,   # Directly downwind, speed is 3 
        210: 3.5, # 210 degrees off the wind, speed is 3.5 
        225: 4,   # 225 degrees off the wind, speed is 4 
        240: 4.8, # 240 degrees off the wind, speed is 4.8 
        270: 5,   # 270 degrees of the wind, speed is 5 
        285: 4,   # 285 degrees off the wind, speed is 4 
        300: 4.5, # 300 degrees off the wind, speed is 4.5 
        315: 4,   # 315 degrees off the wind, speed is 4 
        330: 2.5, # 330 degrees off the wind, speed is 2.5 
        360: 0    # Back to directly into the wind, speed is 0
    }

def newSailingAngleImpl(boat_position, target_position, angle_boat_heading,
                        abs_wind_dir):
    """Determines the best angle to sail at.

        The sailboat follows a locally optimal path (maximize vmg while minimizing
        directional changes) until the global optimum is "better" (based on the
        hysterisis factor).

        Args:
            boat_position (float, float): The global position of the boat.
            target_position (float, float): The global position of the target.
            angle_boat_heading (float): The direction the boat is currently traveling in.
            abs_wind_dir (float): The absolute wind direction.

        Returns:
            float: The best angle to sail (in the global coordinate system).

    """
    beating = 7.0  # TODO what should the beating parameter be?

    boat_to_target = vectorSubtract(boat_position, target_position)
    angle_boat_to_target = vectorAngle(boat_to_target)

    right_angle_max, right_vmg_max = optAngleImpl(angle_boat_to_target,
                                                  abs_wind_dir, True)
    left_angle_max, left_vmg_max = optAngleImpl(angle_boat_to_target,
                                                abs_wind_dir, False)

    hysterisis = 1.0 + (beating / vectorMagnitude(boat_to_target))
    sailing_angle = right_angle_max
    if (abs(right_angle_max - angle_boat_heading) <
            abs(left_angle_max - angle_boat_heading)
            and right_vmg_max * hysterisis < left_vmg_max) or (
                abs(right_angle_max - angle_boat_heading) >=
                abs(left_angle_max - angle_boat_heading)
                and left_vmg_max * hysterisis >= right_vmg_max):
        sailing_angle = left_angle_max

    return sailing_angle


def optAngleImpl(angle_boat_to_target, abs_wind_dir, right):
    """Determines the best angle to sail on either side of the wind.

        The "best angle" maximizes the velocity made good toward the target.

        Args:
            angle_boat_to_target (float): The global angle from the boat to the target.
            abs_wind_dir (float): The absolute wind direction.
            right (bool): True if evaluating the right side of the wind, False for left.

        Returns:
            float: The best angle to sail (in the global coordinate system).
            float: The velocity made good at the best angle.

    """
    delta_alpha = 1.0  # angle resolution
    alpha = 0.0  # potential boat angle relative to absolute wind
    best_vmg = 0.0
    best_angle = abs_wind_dir

    while alpha < 180:
        vel = polarImpl(alpha if right else -1.0 * alpha, abs_wind_dir)
        vmg = dotProduct(vel, unitVector(angle_boat_to_target))

        if vmg > best_vmg:
            best_vmg = vmg
            best_angle = abs_wind_dir + alpha if right else abs_wind_dir - alpha

        alpha = alpha + delta_alpha

    return rangeAngle(best_angle), best_vmg


def polarImpl(angle, abs_wind_dir):
    """Evaluates the polar diagram for a given angle relative to the wind. All values are in degrees.

        Args:
            angle (float): A potential boat heading relative to the absolute wind direction.
            abs_wind_dir (float): The absolute (global) wind direction.

        Returns:
            (float, float): A boat velocity vector (x, y) in the global coordinate system.

    """
    angle = rangeAngle(angle)
    if (angle > 20 and angle < 160) or (angle > 200 and angle < 340):
        # put back into global coords
        return unitVector(angle + abs_wind_dir)
    return 0, 0


def getServoAnglesImpl(abs_wind_dir, yaw, intended_angle):
    """Calculates the sail and rudder angles. All values are in degrees.

        Args:
            abs_wind_dir (float): The absolute (global) wind direction.
            yaw (float): The boat's yaw in the global frame.
            intended_angle (float): The angle the boat intends to travel in the global frame.

        Returns:
            (float, float): A boat velocity vector (x, y) in the global coordinate system.

    """
    sail = 0
    angle_of_attack = abs_wind_dir - yaw
    if abs(angle_of_attack) > 180.0:
        angle_of_attack = (
            (abs(angle_of_attack) - 180) * -1 + 180) * np.sign(angle_of_attack)

    if abs(angle_of_attack) > 160.0:
        sail = 90.0 * np.sign(angle_of_attack)
    elif abs(angle_of_attack) > 110.0:
        sail = 60.0 * np.sign(angle_of_attack)
    elif abs(angle_of_attack) > 75.0:
        sail = 45.0 * np.sign(angle_of_attack)
    elif abs(angle_of_attack) > 35.0:
        sail = 30.0 * np.sign(angle_of_attack)
    else:
        sail = 15.0 * np.sign(angle_of_attack)

    offset = intended_angle - yaw
    #sail = sail + 74.0

    if (abs(offset) < 30):
        tail = offset
    else:
        bigGang = 1
        if (abs(offset) > 180):
            bigGang = -1

        if (offset > 0):
            tail = -30
        if (offset < 0):
            tail = 30
        tail = tail * bigGang

    return sail, tail

def lineFollowing(current_position, point_a, point_b, current_heading, true_wind_dir, tacking_variable, threshold):
    """Calculates the optimal sail and rudder angles based on a line following algorithm.
    
        Args:
            current_position: the current position of the boat
            point_a: the point we are starting from
            point_b: the point we are trying to get to
            current_heading: the current heading of the boat
            true_wind_dir: the true direction of the wind
            tacking_variable: a binary variable {-1,1} to indicate the status of tacking
            threshold: the threshold distance for tacking change
            
        Returns:
            desired_heading: the heading the sailboat should aim for
            rudder_angle: the new angle for the rudder
            sail_angle: the new angle of the sail adjusted for wind and desired heading
    """
    e = calculate_algebraic_distance(current_position, point_a, point_b) #calculate algebraic distance between sailboat and line to be followed
    q = update_tacking_variable(e, threshold, tacking_variable)
    
    line_angle = calculate_line_angle(point_a, point_b, current_heading)
    desired_heading = calculate_desired_heading(line_angle, e)
    
    if isInNoGoZone(desired_heading, true_wind_dir):
        desired_heading = adjustForNoGo(desired_heading, true_wind_dir)
        
    rudder_angle = update_rudder_angle(desired_heading, current_heading)
    sail_angle = update_sail_angle(desired_heading, true_wind_dir)
    
    return desired_heading, rudder_angle, sail_angle

def calculate_algebraic_distance(current_position, point_a, point_b):
    slopeAB = (point_b.y - point_a.y)/(point_b.x - point_a.x)
    x = (-point_a.y + current_position.y + current_position.x/slopeAB + slopeAB*point_a.x) / (1/slopeAB + slopeAB)
    y = slopeAB(x - point_a.x)+point_a.y
    
    if x < point_a.x & x < point_b.x:
        if min(point_a.x, point_b.x) == point_a.x:
            return np.sqrt((current_position.x - point_a.x)^2 + (current_position.y - point_a.y)^2)
        else:
            return np.sqrt((current_position.x - point_b.x)^2 + (current_position.y - point_b.y)^2)
    elif x > point_a.x & x > point_b.x:
        if max(point_a.x, point_b.x) == point_a.x:
            return np.sqrt((current_position.x - point_a.x)^2 + (current_position.y - point_a.y)^2)
        else:
            return np.sqrt((current_position.x - point_b.x)^2 + (current_position.y - point_b.y)^2)
    else:
        return np.sqrt((x - current_position.x)^2 + (y - current_position.y)^2)

def update_tacking_variable(e, threshold, tacking_variable):
    if abs(e) > threshold:
        tacking_variable *= -1 # CHANGE?
        return True
    else:
        return False
    
def calculate_line_angle(point_a, point_b, current_heading):
    return np.arctan2((point_b.y-point_a.y)/(point_b.x-point_a.x))
    
def calculate_desired_heading(line_angle, e, r):
    r = 10 #CHANGE THIS LATER
    return (line_angle - np.atan(e/r))


def isInNoGoZone(desired_heading, true_wind_dir):
    no_go_zone = ?? #do we have this as a variable 
    if abs(desired_heading - true_wind_dir) < no_go_zone:
        return True     
    else:
        return False

def adjustForNoGo(desired_heading, true_wind_dir):
    limit = 20 # Replace num with true limit of wind direction
    if abs(desired_heading - true_wind_dir) < limit:
        if desired_heading < true_wind_dir:
            return desired_heading - 15
        else:
            return desired_heading + 15
    else:
        return desired_heading

def update_rudder_angle(desired_heading, current_heading):
	## Set the rudder angle based on the desired_heading
    ## Need the Dict for polar diagram -> rudder angle (11/15)

    temp = desired_heading - current_heading
    return current_heading + (temp /2) # or is it just return temp?,

def update_sail_angle(desired_heading, true_wind_dir):
    ## Set the sail angle based on desired heading
    ## Need the Dict for polar diagram -> sail angle (11/15)
    return 0

def precisionNavigationImpl(buoys):
    """Generates navigation waypoints from the precision nav buoy locations.

        Args:
            buoys (list of (float, float)): The ordered buoy locations (top_l, top_r, bot_l, bot_r).

        Returns:
            (list of (float, float): The generated waypoint locations.

    """
    # buoys:[topleft_buoy, topright_buoy, botleft_buoy, botright_buoy]
    topleft_buoy = buoys[0]
    topright_buoy = buoys[1]
    botleft_buoy = buoys[2]
    botright_buoy = buoys[3]

    # The number of waypoints to generate along the teardrop curve
    num_waypoints = 10
    
    # Scale the teardrop curve to the correct size
    height = np.sqrt((topleft_buoy[0] - botleft_buoy[0]) ** 2 + 
                     (topleft_buoy[1] - botleft_buoy[1]) **2) / 1.8
    
    width = np.sqrt((botleft_buoy[0] - botright_buoy[0]) ** 2 + 
                    (botleft_buoy[1] - botright_buoy[1]) **2) + 15.0

    t_values = np.linspace(0, 2.0 * np.pi, num_waypoints).tolist()

    # start/finish position between top buoys
    start_pos = midpoint(topleft_buoy, topright_buoy)

    # Find the orientation of the buoys
    def findAngle(bouys):
        topMid = [((bouys[0][0]+bouys[1][0])/2), ((bouys[0][1] + bouys[1][1])/2)]
        bottomMid = [((bouys[2][0]+bouys[3][0])/2), ((bouys[2][1] + bouys[3][1])/2)]
        slopeY = topMid[1] - bottomMid[1]
        slopeX = topMid[0]-bottomMid[0]

        angle = np.arctan2(slopeY,slopeX)
        angle -= np.pi / 2.0 # in the buoy frame, 0 is up, not x-axis
        return angle

    # Get waypoints along the teardrop curve
    def teardrop_shaped_curve(start_pos, tval, height, width, angle=0):
        m = 2
        y = height * np.cos(tval)
        x = width * np.sin(tval) * (np.sin(0.5 * tval) ** m)

        return rotateAndTranslate(x, y, angle, start_pos, height, width)

    # Rotate and translate the positions on the curve to fit the buoys
    def rotateAndTranslate(x, y, angle, start_pos, height, width):
        # get the rotated start position on the parametric curve so we know how to shift
        orig_x = 0
        orig_y = height
        rot_x = orig_x * np.cos(angle) - orig_y * np.sin(angle) 
        rot_y = orig_x * np.sin(angle) + orig_y * np.cos(angle)

        newX = x * np.cos(angle) - y * np.sin(angle) + (start_pos[0] - rot_x)
        newY = x * np.sin(angle) + y * np.cos(angle) + (start_pos[1] - rot_y)
        return (newX, newY)

    out_waypoints = []
    for i in range(len(t_values)):
        out_waypoints.append(teardrop_shaped_curve(start_pos, t_values[i], 
                                                   height, width, 
                                                   findAngle(buoys)))

    return out_waypoints

def collisionAvoidanceImpl(buoys):
    """Generates navigation waypoints from the collision avoidance buoy locations.

        Args:
            buoys (list of (float, float)): The ordered buoy locations (start 2 buoys, then other).

        Returns:
            (list of (float, float): The generated waypoint locations.

    """
    # The first and last waypoints are between the start buoys
    mid = midpoint(buoys[0], buoys[1])
    waypoints = [mid]
    waypoints.append(buoys[2])
    waypoints.append(mid)
    return waypoints


def InnerOuterPoints(start_point, end_point, dist, flag):
    # flag: 1 is in, -1 is out, 0 is on the line
    slope_y = start_point[1] - end_point[1]
    slope_x = start_point[0] - end_point[0]
    theta_slope = np.arctan2(slope_y, slope_x)
    x = start_point[0] + dist * np.cos(theta_slope)
    y = start_point[1] + dist * np.sin(theta_slope)
    if flag != 0:
        x += -flag * 0.1 * dist * np.sin(theta_slope)
        y += flag * 0.1 * dist * np.cos(theta_slope)
    return x, y


def BuoyOffset(start_point, buoy, dist):
    slope_y = start_point[1] - buoy[1]
    slope_x = start_point[0] - buoy[0]
    theta_slope = np.arctan2(slope_y, slope_x)
    x = buoy[0] + dist * np.cos(theta_slope)
    y = buoy[1] + dist * np.sin(theta_slope)
    return x, y


def unitVector(angle):
    x = np.cos(np.deg2rad(angle))
    y = np.sin(np.deg2rad(angle))
    return x, y


def dotProduct(u, v):
    return (u[0] * v[0]) + (u[1] * v[1])


def vectorSubtract(u, v):
    return (u[0] - v[0]), (u[1] - v[1])


def vectorAngle(v):
    return np.rad2deg(np.arctan2(v[1], v[0]))


def vectorMagnitude(v):
    return np.sqrt((v[0]**2) + (v[1]**2))


def rangeAngle(angle):
    return angle % 360


def midpoint(u, v):
    return (u[0] + v[0]) / 2, (u[1] + v[1]) / 2


def dist(u, v):
    return vectorMagnitude(vectorSubtract(u, v))
