import numpy as np
"""
These functions are used by both the simulator and the nav algo.
The simulator cannot access the modules of the nav algo, so this
file should be self contained (i.e. not inherit from any other file).
Everything nav-algo-related that is needed by the simulator should 
exist within this file.
Within this file only, vectors are represented as (x, y) tuples.
"""


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
