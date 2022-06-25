import numpy as np 
import math


def range_by_angle_from_laser_scan(laser_scan_msg, angle):
    """
    Returns nearest measered range to specified angle.

    :param laser_scan_msg: A LaserScan msg from lidar
    :type laser_scan_msg: nav_msgs.LaserScan
    :param angle: Required angle in degrees (-180, 180)
    :type angle: float or int

    :return: Range [m] to the obstacle in the direction specified. None if out of range or scan angle boundaries
    :rtype: float or None
    """

    # Unpack values for convenience
    angle_min = 0.0
    angle_max = float(laser_scan_msg.angle_max) - float(laser_scan_msg.angle_min)
    range_min = float(laser_scan_msg.range_min)
    range_max = float(laser_scan_msg.range_max)
    angle_increment = float(laser_scan_msg.angle_increment)
    distances = np.asarray(laser_scan_msg.ranges)

    if angle < -180 or angle > 180:
        print('Provided angle is out of bounds of [-180, 180]')
        return None
    angle = math.radians(angle+180)



    # Angle probably is in between two points, so will return average of 2 nearest measurements
    angle_id_floor = math.floor((angle - angle_min) / angle_increment)
    angle_id_ceil = math.ceil((angle - angle_min) / angle_increment)

    # Safety precaution
    if angle_id_ceil > len(distances):
        angle_id_ceil = len(distances)

    dist_floor = distances[angle_id_floor]

    dist_ceil = distances[angle_id_ceil]
    dist_avg = (dist_floor + dist_ceil) / 2
    if dist_avg == math.inf:
        print('Distance is at inf range.')
        return None

    return dist_avg

def check_threshold(value_is, value_should, thresh):
    """
    Checks if value_is is within a threshold of value_should.

    :param value_is: measured value.
    :type value_is: float
    :param value_should: base value.
    :type value_should: float
    :param thresh: +- percent from the base value.
    :type thresh: float

    :return: True if within the threshold, False if not
    :rtype: bool
    """

    if abs(value_should - value_is) <= value_should*thresh:
        return True
    else:
        return False

def determine_turn_direction(laser_scan_msg, robot_angle_to_wall, wall_distance_required, distance_threshold=0.1, angles=[90, 45, 0]):
    """
    Determines what type of turn to make. Suitable for 3 measurements.

    :param laser_scan_msg: A LaserScan msg from lidar
    :type laser_scan_msg: nav_msgs.LaserScan
    :param robot_angle_to_wall: relative robot's angle [deg] to the wall. >0 if oriented to a wall, <0 if opposite.
    :type robot_angle_to_wall: float 
    :param wall_distance_required: should-be-distance at first angle (i.e. distance to the wall at 90 deg).
    :type wall_distance_required: float
    :param distance_threshold: determines at what point distance_is equals to  distance_should (0.1 = +-10% of distance_should).
    :type distance_threshold: float
    :param angles: Three angles at which distances are measured (i.e. location of sensors).
    :type angles: list

    :return: Assumed direction of the next turn. 
        "right", "coming_right"
        "left", "coming_left"
        "error" if false range readings
        None if no turn required.

    :rtype: str or None
    """

    robot_angle_to_wall = math.radians(robot_angle_to_wall)

    # d3
    d3_is = range_by_angle_from_laser_scan(laser_scan_msg=laser_scan_msg, angle=angles[0])
    if d3_is is None:
        return "error"
    d3_should = wall_distance_required /  math.cos(robot_angle_to_wall)

    # d2 
    ang_d3_d2 = angles[1] - angles[0]
    ang_d2 = math.radians(robot_angle_to_wall + ang_d3_d2)
    d2_is = range_by_angle_from_laser_scan(laser_scan_msg=laser_scan_msg, angle=angles[1])
    if d2_is is None:
        return "error"
    d2_should = wall_distance_required / math.cos(ang_d2)

    #d1
    d1_is = range_by_angle_from_laser_scan(laser_scan_msg=laser_scan_msg, angle=angles[2])
    if d1_is is None:
        return "error"
    print ("################################")
    print(f'Measured d3: {d3_is}')
    print(f'Calculated d3: {d3_should}')
    print(f'Measured d2: {d2_is}')
    print(f'Calculated d2: {d2_should}')
    print(f'Measured d1: {d1_is}')

    print(f'd3 thresh: {check_threshold(d3_is, d3_should, distance_threshold)}')
    print(f'd2 thresh: {check_threshold(d2_is, d2_should, distance_threshold)}')
    print(f'd1 thresh: {check_threshold(d1_is, wall_distance_required, distance_threshold)}')

    # Cases:
    if check_threshold(d3_is, d3_should, distance_threshold) and check_threshold(d2_is, d2_should, distance_threshold) \
        and d1_is > wall_distance_required * (1.0 + distance_threshold):
        return None

    elif check_threshold(d3_is, d3_should, distance_threshold) and check_threshold(d2_is, d2_should, distance_threshold) \
        and check_threshold(d1_is, wall_distance_required, distance_threshold):
        return "coming_right"

    elif check_threshold(d3_is, d3_should, distance_threshold) \
        and d1_is < wall_distance_required * (1.0 - distance_threshold):
        return "right"

    elif check_threshold(d3_is, d3_should, distance_threshold) and not check_threshold(d2_is, d2_should, distance_threshold) \
        and d1_is > wall_distance_required * (1.0 + distance_threshold):
        return "coming_left"  

    elif not check_threshold(d3_is, d3_should, distance_threshold) and not check_threshold(d2_is, d2_should, distance_threshold) \
        and d1_is > wall_distance_required * (1.0 + distance_threshold):
        return "left"
    
    else:
        print('Case not covered.')
        return "error"


