from msilib.schema import Error
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dumb_wall_follower.methods import *

def scan_callback(laser_scan_msg):
    # rospy.loginfo(f'##### #######')
    # # result = determine_turn_direction(laser_scan_msg, 0, 10)
    # result = range_by_angle_from_laser_scan(laser_scan_msg, 0)
    # rospy.loginfo(f'Result at 0: {result}')
    # result = range_by_angle_from_laser_scan(laser_scan_msg, 45)
    # rospy.loginfo(f'Result at 45: {result}')
    # result = range_by_angle_from_laser_scan(laser_scan_msg, 90)
    # rospy.loginfo(f'Result at 90: {result}')

    result = determine_turn_direction(laser_scan_msg, 0, 0.6)
    rospy.loginfo(f'{result}')

def listener():
    rospy.init_node('laser_scan_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()