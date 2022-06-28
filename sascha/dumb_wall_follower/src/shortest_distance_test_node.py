#! /usr/bin/env python3
from ast import While

from numpy import argmin
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dumb_wall_follower.methods import *
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion, Pose

from tf2_geometry_msgs import PoseStamped
import tf2_ros

import sys

def get_shortest_distance(laser_scan_msg, left_border=100, right_border=-100):
    """
    Determines the shortest distance in the valid area in front of the robot

    :param laser_scan_msg: A LaserScan msg from lidar
    :type laser_scan_msg: nav_msgs.LaserScan
    :param left_border: size of the fov on the left side
    :type left_border: int
    :param right_border: size of the fov on the right side
    :type right_border: int

    return: distance and angle in radians

    :rtype: list
    """
    angle_increment = math.degrees(float(laser_scan_msg.angle_increment))
    distances = np.asarray(laser_scan_msg.ranges)
    len_ = len(distances)
    center_degree = 180

    left_degree = center_degree - left_border
    right_degree = center_degree - right_border

    left_index = int(left_degree / angle_increment)
    right_index = int(right_degree / angle_increment)

    compact_distances = distances[left_index:right_index]

    min_dist_index = np.argmin(compact_distances) + left_index

    angle = angle_increment * min_dist_index
    
    angle = -(angle - 180)

    return [distances[min_dist_index], angle]

def scan_callback(laser_scan_msg):
    dist, angle = get_shortest_distance(laser_scan_msg, 100, -100)
    print(f'distance: {dist}')
    print(f'angle: {angle}')

def listener():
    rospy.init_node('laser_scan_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()