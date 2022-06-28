import numpy as np 
import math
import rospy
from std_msgs.msg import Bool


def pol2cart(d, angle):
    """
    Transforms polar point coordinates to cartesian according to right-hand rule.

    :param d: Distance [m] to point in polar coordinates.
    :type d: float
    :param angle: Angle [rad] from X-axis according to right-hand rule.
    :type angle: float

    :return: Return [x,y] point representation.
    :rtype: [float, float]
    """

    x = d * math.cos(angle)
    y = d * math.sin(angle)
    return [x,y]
    

def distance_between_points(point1, point2):
        """
        Calculates Euclidean distance between two points.

        :param point1: [x,y] coordinates of a point.
        :type point1: list
        :param point2: [x,y] coordinates of a point.
        :type point2: list

        :return: Euclidean distance.
        :rtype: float

        """

        x1, y1 = point1
        x2, y2 = point2
        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        return distance


def project_point_on_line(point, line):
        """
        Calculates a projection of a point onto a line.

        :param point: [x,y] coordinates of a point.
        :type point: list
        :param line: line defined by [x,y] coordinates of two points.
        :type line: list[list, list]

        :return: [x,y] coordinates of the projection and distance between the projection and the input point.
        :rtype: list[list, float]

        """

        x0, y0 = point
        [x1, y1], [x2, y2] = line
        # Case of vertical line:
        if x1 == x2:
            x_inter = x1
            y_inter = y0
            distance = distance_between_points(point, [x_inter, y_inter])
            return [[x_inter, y_inter], distance]

        # Case of horizontal line:
        elif y1 == y2:
            x_inter = x0
            y_inter = y1
            distance = distance_between_points(point, [x_inter, y_inter])
            return [[x_inter, y_inter], distance]

        else:
            # Define line equation: y = ax + b
            a = (y2 - y1) / (x2 - x1)
            b = y1 - a * x1
            # Calculate perpendicular line through the point: y0 = px0 + d
            p = -(1 / a)
            d = y0 - p * x0
            # Get intersection point:
            x_inter = (d - b) / (a - p)
            y_inter = p * x_inter + d

            distance = distance_between_points(point, [x_inter, y_inter])

            return [[x_inter, y_inter], distance]


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


def tunnel_blocker(state: bool):
    p = rospy.Publisher('/tunnel_state', Bool, queue_size=1)
    p.publish(state)