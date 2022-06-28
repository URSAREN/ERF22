import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dumb_wall_follower.methods import *
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion, Pose

from tf2_geometry_msgs import PoseStamped
import tf2_ros

class Robot:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)
        self.name = rospy.get_param('~name')
        self.corner_map = np.asarray(rospy.get_param('~corner_map'))
        self.last_corner_id = None
        self.corner_automaton_state = None

        self.max_linear_speed = rospy.get_param('~max_linear_speed')
        self.max_angular_speed = rospy.get_param('~max_angular_speed')
        self.distance_to_wall = rospy.get_param('~room_distance_to_wall')

        self.angles_left_wall_follow = rospy.get_param('~angles_left_wall_follow')
        self.angles_right_wall_follow = rospy.get_param('~angles_right_wall_follow')
        self.current_wall_location = 'left'

        self.tf_listener = tf.TransformListener()
        
        
        rospy.Subscriber(rospy.get_param('~laser_scan_topic'), LaserScan, self.test) # TODO: connect to main_structure
        self.robot_mover = self.RobotMover()
        rospy.spin()
        

    def range_by_angle_from_laser_scan(self, laser_scan_msg, angle):
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
        angle_max = float(laser_scan_msg.angle_max) - float(laser_scan_msg.angle_min)  # 360
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


    def test(self, laser_scan_msg):
        range, heading = self.get_shortest_distance(laser_scan_msg, left_border=100, right_border=-100)
        print(f'Heading: {math.degrees(heading)}')
        print(f'Distance: {range}')
        # test_pose = self.error2pose(0, 0)
        # self.robot_mover.send_pose_to_move_base(test_pose)




    def main_structure(self, laser_scan_msg):
        """
        Main logic of the robot.
        """
        
        dist, angle = self.get_shortest_distance(laser_scan_msg)
        # orientate to angle
        scan_gap = 30
        dl = self.range_by_angle_from_laser_scan(laser_scan_msg, scan_gap)
        dr = self.range_by_angle_from_laser_scan(laser_scan_msg, -scan_gap)
        if (abs(dl-dr) > 0.2 * dl):
            # turn 180 degree
            dl = self.range_by_angle_from_laser_scan(laser_scan_msg, scan_gap)
            dr = self.range_by_angle_from_laser_scan(laser_scan_msg, -scan_gap)
        # drive to wall (+ 60 cm distance)
        # rotate 90 degree right

        left_turns = 0
        while True:
            turn_direction = self.determine_turn_direction()
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
            d3 = self.range_by_angle_from_laser_scan(laser_scan_msg, self.angles_left_wall_follow[0])
            d2 = self.range_by_angle_from_laser_scan(laser_scan_msg, self.angles_left_wall_follow[1])
            d1 = self.range_by_angle_from_laser_scan(laser_scan_msg, self.angles_left_wall_follow[2])
        elif self.current_wall_location == 'right':
            d3 = self.range_by_angle_from_laser_scan(laser_scan_msg, self.angles_right_wall_follow[0])
            d2 = self.range_by_angle_from_laser_scan(laser_scan_msg, self.angles_right_wall_follow[1])
            d1 = self.range_by_angle_from_laser_scan(laser_scan_msg, self.angles_right_wall_follow[2])
        else:
            raise NotImplementedError('False wall location was given. Only "right" or "left".')

        return [d3, d2, d1]


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
            _, real_distance = project_point_on_line([0, 0], d2d3)
            linear_error = real_distance - self.distance_to_wall
        elif self.current_wall_location == 'right':
            point_d3 = pol2cart(d3_measured, math.radians(self.angles_right_wall_follow[0]))
            point_d2 = pol2cart(d2_measured, math.radians(self.angles_right_wall_follow[1]))
            d2d3 = [point_d3, point_d2] 
            _, real_distance = project_point_on_line([0, 0], d2d3)
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


    def error2pose(self, linear_error, angular_error):
        """
        Transforms errors into a pose in laser coord. frame.

        :param linear_error: Linear distance error from laser frame origin to wall (perpendicular) in [m].
        :type linear_error: float
        :param angular_error: Angular error from laser frame origin to wall in [rad].
        :type angular_error: float

        :return: point in cartesian space.
        :rtype: geometry_msgs/Pose
        
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

        return my_pose


    def determine_turn_direction(self, laser_scan_msg, robot_angle_to_wall, wall_distance_required, distance_threshold=0.1, angles=[90, 45, 0]):
        # TODO: rewrite for new cases (when robot will ride back -> the directions will be switched.) i.e. test if it is already functions.
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

        # d3 related
        d3_is = self.range_by_angle_from_laser_scan(laser_scan_msg=laser_scan_msg, angle=angles[0])
        if d3_is is None:
            return "error"
        d3_should = wall_distance_required /  math.cos(robot_angle_to_wall)
        is_d3_thresh = check_threshold(d3_is, d3_should, distance_threshold)

        # d2 related
        ang_d3_d2 = angles[1] - angles[0]
        ang_d2 = math.radians(robot_angle_to_wall + ang_d3_d2)
        d2_is = self.range_by_angle_from_laser_scan(laser_scan_msg=laser_scan_msg, angle=angles[1])
        if d2_is is None:
            return "error"
        d2_should = wall_distance_required / math.cos(ang_d2)
        is_d2_thresh = check_threshold(d2_is, d2_should, distance_threshold)

        #d1 related
        d1_is = self.range_by_angle_from_laser_scan(laser_scan_msg=laser_scan_msg, angle=angles[2])

        if d1_is is None:
            return "error"

        # Cases:
        if is_d3_thresh and is_d2_thresh and d1_is > wall_distance_required * (1.0 + distance_threshold):
            return None

        elif is_d3_thresh and is_d2_thresh and check_threshold(d1_is, wall_distance_required, distance_threshold):
            return "coming_right"

        elif is_d3_thresh and d1_is < wall_distance_required * (1.0 - distance_threshold):
            return "right"

        elif is_d3_thresh and not is_d2_thresh and d1_is > wall_distance_required * (1.0 + distance_threshold):
            return "coming_left"  

        elif not is_d3_thresh and not is_d2_thresh and d1_is > wall_distance_required * (1.0 + distance_threshold):
            return "left"
        
        else:
            print('Case not covered.')
            return "error"


    def get_shortest_distance(self, laser_scan_msg, left_border=90, right_border=-90):
        """
        Determines the shortest distance in the valid area in front of the robot

        :param laser_scan_msg: A LaserScan msg from lidar
        :type laser_scan_msg: nav_msgs.LaserScan
        :param left_border: left fov angle in [deg]
        :type left_border: int
        :param right_border: right fov angle in [deg]
        :type right_border: int

        return: distance and angle in [rad]

        :rtype: list
        """

        angle_increment = float(laser_scan_msg.angle_increment)
        distances = np.asarray(laser_scan_msg.ranges)

        angle_diff_left = math.radians(180 - left_border)
        angle_diff_right = math.radians(180 - right_border)

        angle_id_left = int(angle_diff_left / angle_increment)
        angle_id_right = int(angle_diff_right / angle_increment)

        min_range = min(distances[angle_id_left:angle_id_right])
        min_id = np.argmin(distances[angle_id_left:angle_id_right]) + angle_id_left
        min_angle = min_id * angle_increment - math.radians(180)

        return [min_range, min_angle]


    class RobotMover:
        def __init__(self):
            self.goal_pose_publisher = rospy.Publisher(rospy.get_param('~publish_pose_to_topic'), PoseStamped, queue_size=10)

        
        def send_pose_to_move_base(self, goal_pose, goal_pose_frame='laser'):
            """
            Sends pose to move_base to follow.

            :param goal_pose: Pose for the robot to move to expressed in its own frame.
            :type goal_pose: geometry_msgs/Pose
            :param goal_pose_frame: Frame in which goal_pose is expressed.
            :type goal_pose_frame: str

            """

            my_pose_in_base = self.transform_pose(input_pose=goal_pose, from_frame=goal_pose_frame, to_frame='map')

            pose_stamped = PoseStamped()

            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.pose = my_pose_in_base

            self.goal_pose_publisher.publish(pose_stamped)


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
                pass

            

if __name__ == '__main__':
    robotA = Robot()
    