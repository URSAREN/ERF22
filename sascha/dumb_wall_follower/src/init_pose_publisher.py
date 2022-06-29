#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import LaserScan
from dumb_wall_follower.methods import *
from tf2_geometry_msgs import PoseStamped

def callback(msg):
    print('Hi from init pose publisher.')
    init_pose = PoseStamped()
    init_pose.header.frame_id = rospy.get_param('~initial_pose_frame')
    init_pose.header.stamp = rospy.Time.now()
    init_pose.pose.orientation.w = 1.0
    initial_pose_publisher = rospy.Publisher(rospy.get_param('~initial_pose_topic'), PoseStamped, queue_size=10)
    initial_pose_publisher.publish(init_pose)


if __name__ == '__main__':
    rospy.init_node('init_pose_publisher', anonymous=True)
    rospy.Subscriber(rospy.get_param('~laser_scan_topic'), LaserScan, callback)
    rospy.spin()
