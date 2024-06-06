#!/usr/bin/env python
# coding=utf-8
"""
We will move your robot!

Make sure the robot is on the ground, there are no cables around and that there is nothing in front
of the robot!
"""
import math
import traceback
from datetime import datetime
import rospy
import time
import numpy as np
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal

from scipy.signal import find_peaks

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

other_robot_position = None

num_lidar = 0


def lidar_callback(data):
    # type: (LaserScan) -> None
    global other_robot_position, num_lidar
    num_lidar += 1

    measurements_per_degree = len(data.ranges) / 360.0



    ranges = np.array(data.ranges)

    last = 0
    for i in range(len(ranges)):
        if math.isinf(ranges[i]):
            ranges[i] = last
        else:
            last = ranges[i]

    peaks, properties = find_peaks(
        -ranges,
        prominence=0.15,
        height=(-5, -0.1),
        #threshold=0.005,
        width=(
            4,
            160
        ),
        plateau_size=(0, 200)
    )

    best_dist = None
    best_index = None

    debug = True

    if debug:
        print "##################################"
    # print properties
    for peak_index in range(len(peaks)):
        peak = peaks[peak_index]
        left_edge = properties["left_edges"][peak_index]
        right_edge = properties["right_edges"][peak_index]
        height = properties["peak_heights"][peak_index]
        width = properties["widths"][peak_index]

        m_width = calculate_m_width(
            0,
            int(width),
            height,
            height,
            measurements_per_degree
        )

        if not 0.04 < m_width < 0.08:
            continue

        if debug:
            print peak,

            for key in properties.keys():
                print " " + key[:-1] + ": " + "{:.2f}".format(properties[key][peak_index]),

            print m_width

        best_index = peak
        best_dist = height

    if best_dist:
        dx, dy = calculate_other_robot_position(best_index / measurements_per_degree, best_dist)
        other_robot_position = (dx, dy, time.time())

    if debug:
        print "##################################"

    return None

    threshold = 0.1

    last_index = 0
    last_distance = 0

    a = -1
    a_diff = 0

    # other_robot_position = None

    print "## START"
    for i in range(len(data.ranges)):
        distance = data.ranges[i]

        if math.isinf(distance):
            continue
        # if distance > 1000000:
        #    continue

        if last_distance != 0:

            difference_per_index = (distance - last_distance)  # / (i - last_index)
            if difference_per_index < -threshold:
                # print "A:", last_index, i, " difference_per_index:", difference_per_index
                a = i
                a_diff = difference_per_index

            if difference_per_index > threshold:
                # print "B:", last_index, i, " difference_per_index:", difference_per_index

                b = last_index

                if a != -1:
                    distance_between_a_and_b = calculate_m_width(
                        a,
                        b,
                        data.ranges[a],
                        data.ranges[b],
                        measurements_per_degree
                    )

                    if 0.04 < distance_between_a_and_b < 0.12:
                        # print data.ranges[a] - data.ranges[b]

                        other_robot_distance = (data.ranges[a] + data.ranges[b]) / 2
                        other_robot_angle = \
                            (
                                    a / measurements_per_degree +
                                    b / measurements_per_degree
                            ) / 2

                        dx, dy = calculate_other_robot_position(other_robot_angle, other_robot_distance)

                        print "suspecting robot between", a / measurements_per_degree, "and", b / measurements_per_degree, ":", distance_between_a_and_b
                        print "x:", dx, "y:", dy
                        other_robot_position = (dx, dy, time.time())
                        # print a_diff
                        # print difference_per_index

        last_distance = distance
        last_index = i


def calculate_other_robot_position(other_robot_angle, other_robot_distance):
    dx = math.cos(math.radians(other_robot_angle)) * other_robot_distance
    dy = math.sin(math.radians(other_robot_angle)) * other_robot_distance
    return dx, dy


def calculate_m_width(index_a, index_b, range_a, range_b, measurements_per_degree):
    angle = math.radians((index_b - index_a) / measurements_per_degree)
    distance_between_a_and_b = \
        math.sqrt(
            range_a ** 2 + range_b ** 2
            - 2 * range_a * range_b * math.cos(angle)
        )
    return distance_between_a_and_b


def get_other_robot():
    global other_robot_position
    return other_robot_position


if __name__ == "__main__":
    rospy.init_node('air_challenge_2', disable_signals=True)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)

    start_time = datetime.now()
    print("Starting at {}".format(start_time))

    while True:
        time.sleep(1)
