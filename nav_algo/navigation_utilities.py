import numpy as np
"""
These functions are used by both the simulator and the nav algo.
The simulator cannot access the modules of the nav algo, so this
file should be self contained (i.e. not inherit from any other file).
Everything nav-algo-related that is needed by the simulator should 
exist within this file.
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