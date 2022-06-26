#! /usr/bin/env python3
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dumb_wall_follower.methods import *

distance_to_wall = 0.6
angles_left_wall_follow = [-90, -45, 0] # d3, d2, d1 in deg
angles_right_wall_follow = [90, 45, 0] # d3, d2, d1 in deg
current_wall_location = 'left'

def measure_distances_to_wall(laser_scan_msg):
        """
        Returns measured distances to the wall.

        :param laser_scan_msg: A LaserScan msg from lidar
        :type laser_scan_msg: nav_msgs.LaserScan

        :return: Return measured d3, d2, d1 ranges.
        :rtype: list
        """

        if current_wall_location == 'left':
            d3 = range_by_angle_from_laser_scan(laser_scan_msg, angles_left_wall_follow[0])
            d2 = range_by_angle_from_laser_scan(laser_scan_msg, angles_left_wall_follow[1])
            d1 = range_by_angle_from_laser_scan(laser_scan_msg, angles_left_wall_follow[2])
        elif current_wall_location == 'right':
            d3 = range_by_angle_from_laser_scan(laser_scan_msg, angles_right_wall_follow[0])
            d2 = range_by_angle_from_laser_scan(laser_scan_msg, angles_right_wall_follow[1])
            d1 = range_by_angle_from_laser_scan(laser_scan_msg, angles_right_wall_follow[2])
        else:
            raise NotImplementedError('False wall location was given. Only "right" or "left".')

        return [d3, d2, d1]

def calculate_relative_angle_to_wall(d3_measured, d2_measured):
        """
        Calculates relative orientation angle to a wall.

        Assumes both laser rays track the same wall.

        :param d3_measured: scan distance in d3 direction.
        :type d3_measured: float
        :param d2_measured: scan distance in d2 direction.
        :type d2_measured: float

        :return: relative orientation in radians.
        :rtype: float
        """

        ang_d2_ideal = angles_left_wall_follow[1] - angles_left_wall_follow[0] # difference between d3 and d2
        d2_ideal = distance_to_wall / math.cos(ang_d2_ideal)
        d3_ideal = distance_to_wall

        if current_wall_location == 'left':
            if d3_ideal <= d3_measured <= d2_ideal and d2_measured >= d2_ideal:
                return math.acos(distance_to_wall / d3_measured)
            elif d3_measured >= d3_ideal and d2_ideal <= d2_measured <= d3_ideal:
                return -math.acos(distance_to_wall / d3_measured)
            else:
                return None

        elif current_wall_location == 'right':
            if d3_ideal <= d3_measured <= d2_ideal and d2_measured >= d2_ideal:
                return -math.acos(distance_to_wall / d3_measured)
            elif d3_measured >= d3_ideal and d2_ideal <= d2_measured <= d3_ideal:
                return math.acos(distance_to_wall / d3_measured)
            else:
                return None

        else:
            raise NotImplementedError('False wall location was given. Only "right" or "left".')

def scan_callback(laser_scan_msg):
    # rospy.loginfo(f'##### #######')
    # # result = determine_turn_direction(laser_scan_msg, 0, 10)
    # result = range_by_angle_from_laser_scan(laser_scan_msg, 0)
    # rospy.loginfo(f'Result at 0: {result}')
    # result = range_by_angle_from_laser_scan(laser_scan_msg, 45)
    # rospy.loginfo(f'Result at 45: {result}')
    # result = range_by_angle_from_laser_scan(laser_scan_msg, 90)
    # rospy.loginfo(f'Result at 90: {result}')

    rospy.loginfo(f'#################')
    result = determine_turn_direction(laser_scan_msg, 0, 0.6)
    rospy.loginfo(f'{result}')
    d3_is, d2_is, d1_is = measure_distances_to_wall(laser_scan_msg)
    result = calculate_relative_angle_to_wall(d3_is,d2_is)
    rospy.loginfo(f'{result}')

def listener():
    rospy.init_node('laser_scan_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()