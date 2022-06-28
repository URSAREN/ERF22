from re import I
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
        self.corner_map = np.asarray(rospy.get_param('~corner_map'))
        self.last_corner_id = None
        self.corner_automaton_state = None

        self.max_linear_speed = rospy.get_param('~max_linear_speed') # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed')  # rad/s
        self.distance_to_wall = rospy.get_param('~distance_to_wall') # m
        self.sensor_position = 0  # distance from sensor's center to robot's center in [m]

        self.angles_left_wall_follow = [90, 60, 0] # d3, d2, d1 in deg
        self.angles_right_wall_follow = [-90, -50, 0] # d3, d2, d1 in deg
        self.current_wall_location = 'left'

        rospy.init_node(self.name, anonymous=True)
        self.tf_listener = tf.TransformListener()
        
        self.goal_pose_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped)

    def init_listener(self):
        """
        
        """
        rospy.Subscriber("/scan", LaserScan, self.main_structure)
        rospy.spin()

    def get_to_initial_pose(self, laser_scan_msg):
        # dist, angle = get_shortest_distance(laser_scan_msg)
        dist = 0
        angle = math.radians(90)
        x_shortest, y_shortest = pol2cart(dist, angle)

        my_pose = Pose()


        my_pose.position.x = x_shortest
        my_pose.position.y = y_shortest
        my_pose.position.z = 0

        q = quaternion_from_euler(0, 0, angle)
        my_pose.orientation.x = q[0]
        my_pose.orientation.y = q[1]
        my_pose.orientation.z = q[2]
        my_pose.orientation.w = q[3]

        self.goal2move_base(my_pose)



    def main_structure(self, laser_scan_msg):
        """
        Main logic of the robot.
        """
        
        dist, angle = get_shortest_distance(laser_scan_msg)
        # orientate to angle
        scan_gap = 30
        dl = range_by_angle_from_laser_scan(laser_scan_msg, scan_gap)
        dr = range_by_angle_from_laser_scan(laser_scan_msg, -scan_gap)
        if (abs(dl-dr) > 0.2 * dl):
            # turn 180 degree
            dl = range_by_angle_from_laser_scan(laser_scan_msg, scan_gap)
            dr = range_by_angle_from_laser_scan(laser_scan_msg, -scan_gap)
        # drive to wall (+ 60 cm distance)
        # rotate 90 degree right

        left_turns = 0
        while True:
            turn_direction = determine_turn_direction()
            if turn_direction == 'left':
                if left_turns == 0:
                    self.last_corner_id = 3
                    tunnel_blocker(True)
                left_turns += 1
                pass
            elif turn_direction == 'coming_right':
                tunnel_blocker(False)
                pass
            elif turn_direction == 'right':
                pass
            elif turn_direction == 'coming_left':
                # reduce speed
                pass
            else:
                pass
            d3, d2, d1 = self.measure_distance_to_wall()
            lin_error, ang_error = self.calculate_position_error_laser_frame(d3,d2)
            wall_move_goal = self.laser2base(lin_error,ang_error)
            #send to movebase
            #update cornermap


        #Code abstract:
        """
        1. Initialization
        2. Scan and find a point with the smallest range (in the valid area in front of the robot.)
        3. Orient itself to this point.
        4. Do values from d1 +- 30 deg  ~equal?
            4.1 Yes -> wall found -> move to this point(up untill safe distance to wall).
            4.2 No -> exception -> rotate 180 deg left -> go to 2.
        5. Rotate 90 deg right.
        6. Initial wall following
            6.0 determine_turn_direction()
                6.0.1 If None -> jump to 6.1
                6.0.2 If coming_right -> reduce speed -> jump to 6.1
                6.0.3 if right -> stop -> rotate 90 deg right -> jump to 6.
                6.0.4 if coming_left -> reduce speed -> dumb PoseGoal (directly in front) for move_base -> jump to 6.0.5
                6.0.5 while not left -> dumb PoseGoal (directly in front) with reduced speed. -> 6.0.6
                6.0.6 Rotate 90 deg left around edge (move base command possible??). -> 6.0.7
                6.0.7 Update internal corner map location. -> 7.
            6.1 measure_distances_to_wall() for d3 left and d2 left.
            6.2 calculate_position_error_laser_frame().
            6.3 laser2base()
            6.4 Send point from 6.3 to move_base.
        7. Send True to /robotA_in_tunnel
        8. Wall following
            8.0 Check internal corner map location -> ??????

            8.1 determine_turn_direction()
                8.1.0 If None -> 8.2
                8.1.1 If coming_left -> reduce speed -> dump PoseGoal (directly in front) for move_base
                8.1.2 If coming_right -> reduce speed -> 8.2
                8.1.3 If left -> stop -> rotate 90 deg left around edge -> update internal corner map location -> 8.
                8.1.4 If right -> stop -> rotate 90 deg right -> update internal corner map location -> 8.
        
            8.2 measure_distances_to_wall() for d3 left and d2 left.
            8.2 calculate_position_error_laser_frame().
            8.3 laser2base()
            8.4 Send point from 6.3 to move_base
        """        

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


    
    # def adjust_parallel_to_wall(self, laser_scan_msg):
    #     """
    #     Adjust the robot velocities to reduce difference between d3 and d2.

    #     :param laser_scan_msg: A LaserScan msg from lidar
    #     :type laser_scan_msg: nav_msgs.LaserScan

    #     """
        
    #     d3_measured, d2_measured, d1_measured = self.measure_distances_to_wall(laser_scan_msg)
    #     relative_angle = self.calculate_relative_angle_to_wall(d3_measured, d2_measured)
    #     linear_error, angular_error = self.calculate_position_error(d3_measured, d2_measured)





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

    def laser2base_error(self, linear_error, angular_error):
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
        return transformed_pose
    
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

    def goal2move_base(self, goal_pose_in_robot_frame):
        """
        Send dumb command to move_base to test if it is working.

        :param goal_pose_in_robot_frame: Pose for the robot to move to.
        :type goal_pose_in_robot_frame: geometry_msgs/Pose


        """


        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose = goal_pose_in_robot_frame
        print(f'Sending following to move_base: {pose_stamped}')

        # rospy.init_node('dumb_command_publisher', anonymous=True)
        # goal_pose_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped)
        my_twist = Twist()
        my_twist.angular.z = 0.5
        self.goal_pose_publisher.publish(pose_stamped)
    
        
if __name__ == '__main__':
    corner_map_a = rospy.get_param('/corner_map')
    robotA = Robot()
    robotA.initiate_listener()