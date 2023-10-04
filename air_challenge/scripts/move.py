#!/usr/bin/env python
# coding=utf-8
"""
We will move your robot!

Make sure the robot is on the ground, there are no cables around and that there is nothing in front
of the robot!
"""

from datetime import datetime

import rospy
from geometry_msgs.msg import Twist


def check_stop(start_time, seconds=2):
    """
    Check if the robot should stop depending on the time elapsed in seconds.
    """
    now = datetime.now()
    delta = now - start_time
    return (delta.seconds) > seconds


if __name__=="__main__":

    # Create a ROS node for our package
    rospy.init_node('air_challenge')
    # Our package does not rely on any input information (it uses time), we output wheel command
    # messages. We register a publisher for the appropriate topic and message type.
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

    # Our robot has a fixed speed
    speed = 0.2 # m/s

    start_time = datetime.now()
    print("Starting at {}".format(start_time))

    try:
        # Keep driving for a predefined period of time by repeatedly publishing the move message
        while not check_stop(start_time):
            print("  Moving forward!")
            twist = Twist()
            twist.linear.x = speed;
            pub.publish(twist)

    except Exception as e:
        print(e)

    # On exit, make sure we stop the robot
    finally:
        print("Stopping..")
        twist = Twist()
        twist.linear.x = 0;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)


