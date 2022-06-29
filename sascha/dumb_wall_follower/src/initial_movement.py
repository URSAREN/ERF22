#! /usr/bin/env python3

import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from sensor_msgs.msg import LaserScan
from dumb_wall_follower.methods import *
from tf.transformations import quaternion_from_euler
import math


# create the connection to the action server
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
# waits until the action server is up and running
client.wait_for_server()

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    print('[Feedback] Going to Goal Pose...')


def subscriber_callback(msg):
    dist, angle = get_shortest_distance(msg)
    # transform dist to robot koordinatesystem?
    
    ### Initialize the Rotation towards the wall
    # -----------------------------------------
    q = quaternion_from_euler(0, 0, angle)
    # creates a goal to send to the action server
    goal_1 = MoveBaseGoal()
    goal_1.target_pose.header.frame_id = 'map'
    goal_1.target_pose.pose.position.x = 0.00
    goal_1.target_pose.pose.position.y = 0.00
    goal_1.target_pose.pose.position.z = 0.00
    goal_1.target_pose.pose.orientation.x = q[0]
    goal_1.target_pose.pose.orientation.y = q[1]
    goal_1.target_pose.pose.orientation.z = q[2]
    goal_1.target_pose.pose.orientation.w = q[3]
    # sends the goal to the action server, specifying which feedback function
    # to call when feedback received
    client.send_goal(goal_1, feedback_cb=feedback_callback)
    # wait until the result is obtained
    client.wait_for_result()
    rospy.loginfo("Robot performed first goal")


    ### Initialize the movement towards the wall
    # -----------------------------------------
    distance = dist - 0.6 - 0.5 # the distance is the measured distance, minus the 0.6m security distance, minus half the diameter of the robot
    # creates a goal to send to the action server
    goal_2 = MoveBaseGoal()
    goal_2.target_pose.header.frame_id = 'map'
    goal_2.target_pose.pose.position.x = distance
    goal_2.target_pose.pose.position.y = 0.00
    goal_2.target_pose.pose.position.z = 0.00
    goal_2.target_pose.pose.orientation.x = 0.00
    goal_2.target_pose.pose.orientation.y = 0.00
    goal_2.target_pose.pose.orientation.z = 0.00
    goal_2.target_pose.pose.orientation.w = 0.00
    # sends the goal to the action server, specifying which feedback function
    # to call when feedback received
    client.send_goal(goal_2, feedback_cb=feedback_callback)
    # wait until the result is obtained
    client.wait_for_result()
    rospy.loginfo("Robot performed second goal")

    ### Initialize the 90 degree turn to be parallel to the wall
    # -----------------------------------------
    q = quaternion_from_euler(0, 0, math.radians(90))
    # creates a goal to send to the action server
    goal_3 = MoveBaseGoal()
    goal_3.target_pose.header.frame_id = 'map'
    goal_3.target_pose.pose.position.x = 0.00
    goal_3.target_pose.pose.position.y = 0.00
    goal_3.target_pose.pose.position.z = 0.00
    goal_3.target_pose.pose.orientation.x = q[0]
    goal_3.target_pose.pose.orientation.y = q[1]
    goal_3.target_pose.pose.orientation.z = q[2]
    goal_3.target_pose.pose.orientation.w = q[3]
    # sends the goal to the action server, specifying which feedback function
    # to call when feedback received
    client.send_goal(goal_3, feedback_cb=feedback_callback)
    # wait until the result is obtained
    client.wait_for_result()
    rospy.loginfo("Robot performed third goal")


def listener():
    # initializes the initial movement node (action client node + subscriber)
    rospy.init_node('initial_movement')
    # intialize subscriber
    sub = rospy.Subscriber('/scan', LaserScan, subscriber_callback)
    # keep it running
    # rospy.spin() # we do not need it to be running endlessly


if __name__ == '__main__':
    listener()