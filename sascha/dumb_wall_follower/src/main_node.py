from msilib.schema import Error
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dumb_wall_follower.methods import *

class Robot():
    def __init__(self, name, corner_map):
        self.name = name
        self.corner_map = corner_map
        self.last_corner_id = None
        self.corner_automaton_state = None

        self.node = rospy.init_node(self.name, anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.max_linear_speed = 0.2 # m/s
        self.max_angular_speed = 0.1 # rad/s
        self.distance_to_wall = 0.6 # m

        self.angles_left_wall_follow = [-90, -45, 0] # d3, d2, d1 in deg
        self.angles_right_wall_follow = [90, 45, 0] # d3, d2, d1 in deg
        self.current_wall_location = 'left'
        

    def measure_distances_to_wall(self, laser_scan_msg):
        """
        Returns measured distances to the wall.

        :param laser_scan_msg: A LaserScan msg from lidar
        :type laser_scan_msg: nav_msgs.LaserScan

        :return: Return measured d3, d2, d1 ranges.
        :rtype: list
        """

        if self.current_wall_location == 'left':
            d3 = range_by_angle_from_laser_scan(laser_scan_msg, self.angles_left_wall_follow[0])
            d2 = range_by_angle_from_laser_scan(laser_scan_msg, self.angles_left_wall_follow[1])
            d1 = range_by_angle_from_laser_scan(laser_scan_msg, self.angles_left_wall_follow[2])
        elif self.current_wall_location == 'right':
            d3 = range_by_angle_from_laser_scan(laser_scan_msg, self.angles_right_wall_follow[0])
            d2 = range_by_angle_from_laser_scan(laser_scan_msg, self.angles_right_wall_follow[1])
            d1 = range_by_angle_from_laser_scan(laser_scan_msg, self.angles_right_wall_follow[2])
        else:
            raise NotImplementedError('False wall location was given. Only "right" or "left".')

        return [d3, d2, d1]


    
    def adjust_parallel_to_wall(self, laser_scan_msg):
        """
        Adjust the robot velocities to reduce difference between d3 and d2.

        :param laser_scan_msg: A LaserScan msg from lidar
        :type laser_scan_msg: nav_msgs.LaserScan

        """
        d3_is, d2_is, d1_is = self.measure_distances_to_wall(laser_scan_msg)
        relative_angle = self.



    def calculate_relative_angle_to_wall(self, d3_measured, d2_measured):
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

        ang_d2_ideal = self.angles_left_wall_follow[1] - self.angles_left_wall_follow[0] # difference between d3 and d2
        d2_ideal = self.distance_to_wall / math.cos(ang_d2_ideal)
        d3_ideal = self.distance_to_wall

        if self.current_wall_location == 'left':
            if d3_ideal <= d3_measured <= d2_ideal:
                return math.acos(self.distance_to_wall / d3_measured)
            elif d3_measured >= d3_ideal and d2_ideal <= d2_measured <= d3_ideal:
                return -math.acos(self.distance_to_wall / d3_measured)
            else:
                raise NotImplementedError('Rotation angle is too big.')

        elif self.current_wall_location == 'right':
            if d3_ideal <= d3_measured <= d2_ideal:
                return -math.acos(self.distance_to_wall / d3_measured)
            elif d3_measured >= d3_ideal and d2_ideal <= d2_measured <= d3_ideal:
                return math.acos(self.distance_to_wall / d3_measured)
            else:
                raise NotImplementedError('Rotation angle is too big.')

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

        result = determine_turn_direction(laser_scan_msg, 0, 0.6)
        rospy.loginfo(f'{result}')
def listener():
    rospy.init_node('laser_scan_listener', anonymous=True)

    rospy.Publisher('/cmd_vel', LaserScan)
    
    rospy.spin()

def main():
    """
    Determines the behaviour of the robot A.
    """


if __name__ == '__main__':
    listener()