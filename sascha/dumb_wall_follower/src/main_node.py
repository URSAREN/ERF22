import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dumb_wall_follower.methods import *
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion, Pose

from tf2_geometry_msgs import PoseStamped
import tf2_ros

class Robot():
    def __init__(self, name, corner_map):
        self.name = name
        self.corner_map = corner_map
        self.last_corner_id = None
        self.corner_automaton_state = None

        # self.node = rospy.init_node(self.name, anonymous=True)
        # self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        # self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.max_linear_speed = 0.2 # m/s
        self.max_angular_speed = 0.1 # rad/s
        self.distance_to_wall = 0.4 # m
        self.sensor_position = 0  # distance from sensor's center to robot's center in [m]

        self.angles_left_wall_follow = [90, 60, 0] # d3, d2, d1 in deg
        self.angles_right_wall_follow = [-90, -50, 0] # d3, d2, d1 in deg
        self.current_wall_location = 'left'

        self.tf_listener = tf.TransformListener()
        

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
        
        d3_measured, d2_measured, d1_measured = self.measure_distances_to_wall(laser_scan_msg)
        relative_angle = self.calculate_relative_angle_to_wall(d3_measured, d2_measured)
        linear_error, angular_error = self.calculate_position_error(d3_measured, d2_measured)





    def calculate_position_error_laser_frame(self, d3_measured, d2_measured):
        """
        Calculates both errors: linear to wall and angular to wall.

        linear_error > 0 if too close to wall (i.e. increase distance); and < 0 if too far (i.e. decrease distance).
        angular_error > 0 if from to wall (i.e. rotate in positive z); and < 0 if to wall (i.e. rotate in negative z).

        :param d3_measured: scan distance in d3 direction.
        :type d3_measured: float
        :param d2_measured: scan distance in d2 direction.
        :type d2_measured: float

        :return: Linear error [m] and angular error [rad].
        :rtype: list
        """
        if self.current_wall_location == 'left':
            point_d3 = pol2cart(d3_measured, math.radians(self.angles_left_wall_follow[0]))
            point_d2 = pol2cart(d2_measured, math.radians(self.angles_left_wall_follow[1]))
            d2d3 = [point_d3, point_d2] 
            _, real_distance = project_point_on_line([-self.sensor_position, 0], d2d3)
            linear_error = real_distance - self.distance_to_wall
        elif self.current_wall_location == 'right':
            point_d3 = pol2cart(d3_measured, math.radians(self.angles_right_wall_follow[0]))
            point_d2 = pol2cart(d2_measured, math.radians(self.angles_right_wall_follow[1]))
            d2d3 = [point_d3, point_d2] 
            _, real_distance = project_point_on_line([-self.sensor_position, 0], d2d3)
            linear_error = self.distance_to_wall - real_distance


        angular_error = self.calculate_relative_angle_to_wall(d3_measured, d2_measured) # angle should be 0

        return [linear_error, angular_error]


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

        if self.current_wall_location == 'left':
            point_d3 = pol2cart(d3_measured, math.radians(self.angles_left_wall_follow[0]))
            point_d2 = pol2cart(d2_measured, math.radians(self.angles_left_wall_follow[1]))
            ang_d3_d2 = math.radians(self.angles_left_wall_follow[0] - self.angles_left_wall_follow[1])
            print(f'Point d3: {point_d3}')
            print(f'Point d2: {point_d2}')
            d2d3 = math.sqrt(d3_measured ** 2 + d2_measured ** 2 - 2 * d2_measured * d3_measured * math.cos(ang_d3_d2)) # cosine theorem
            if point_d3[1] >= point_d2[1]: # acute angle
                alpha = math.asin(d2_measured * math.sin(ang_d3_d2) / d2d3) # sine theorem
            else: # obtuse angle
                alpha = math.radians(180) - math.asin(d2_measured * math.sin(ang_d3_d2) / d2d3)
            print(f'Alpha from sine theorem: {math.degrees(alpha)}')
            alpha -= math.radians(90)
            return alpha

        elif self.current_wall_location == 'right':
            point_d3 = pol2cart(d3_measured, math.radians(self.angles_right_wall_follow[0]))
            point_d2 = pol2cart(d2_measured, math.radians(self.angles_right_wall_follow[1]))
            ang_d3_d2 = math.radians(self.angles_right_wall_follow[1] - self.angles_right_wall_follow[0])
            print(f'Point d3: {point_d3}')
            print(f'Point d2: {point_d2}')
            d2d3 = math.sqrt(d3_measured ** 2 + d2_measured ** 2 - 2 * d2_measured * d3_measured * math.cos(ang_d3_d2)) # cosine theorem
            if point_d3[1] <= point_d2[1]: # acute angle
                alpha = math.asin(d2_measured * math.sin(ang_d3_d2) / d2d3) # sine theorem
            else: # obtuse angle
                alpha = math.radians(180) - math.asin(d2_measured * math.sin(ang_d3_d2) / d2d3)
            print(f'Alpha from sine theorem: {math.degrees(alpha)}')
            alpha = math.radians(90) - alpha
            return alpha
        
        else:
            raise NotImplementedError('False wall location was given. Only "right" or "left".')

    def laser2base(self, linear_error, angular_error):
        """
        Converts linear and angular errors from laser to base_link frame.

        :param linear_error: Linear distance error from laser frame origin to wall (perpendicular) in [m].
        :type linear_error: float
        :param angular_error: Angular error from laser frame origin to wall in [rad].
        :type angular_error: float

        :return: Recalculated errors in base_link frame [base_lin_error, base_ang_error].
        :rtype: list[float, float]
        """
        my_pose = Pose()



        my_pose.position.y = linear_error * math.cos(-angular_error)
        my_pose.position.x = linear_error * math.sin(-angular_error)
        my_pose.position.z = 0
        q = quaternion_from_euler(0, 0, angular_error)
        my_pose.orientation.x = q[0]
        my_pose.orientation.y = q[1]
        my_pose.orientation.z = q[2]
        my_pose.orientation.w = q[3]

        transformed_pose = self.transform_pose(my_pose, 'laser', 'base_link')
        print (f"Position of the error in the base_link: {transformed_pose}")

    
    @staticmethod
    def transform_pose(input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
        



def scan_callback(laser_scan_msg):
    robotA = Robot(name='RobotA', corner_map=[])
    d3_measured, d2_measured, d1_measured = robotA.measure_distances_to_wall(laser_scan_msg)
    print('######################')
    print(f'd3 measured is: {d3_measured}')
    print(f'd2 measured is: {d2_measured}')
    print(f'd1 measured is: {d1_measured}')
    ang_to_wall = robotA.calculate_relative_angle_to_wall(d3_measured, d2_measured)
    print(f'Relative angle to wall is: {math.degrees(ang_to_wall)}')
    lin_error, ang_error = robotA.calculate_position_error_laser_frame(d3_measured, d2_measured)
    print(f'Linear position error LASER is: {lin_error}')
    print(f'Angular position error LASER is: {math.degrees(ang_error)}')

    robotA.laser2base(lin_error, ang_error)

    





    

def listener():
    rospy.init_node('laser_scan_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()