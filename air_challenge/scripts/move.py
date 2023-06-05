#!/usr/bin/env python
# coding=utf-8

import rospy

from geometry_msgs.msg import Twist

import time
import sys, select, termios, tty

msg = """
We will move your robot!

Make sure there is nothing in front of the robot.
"""


if __name__=="__main__":

    rospy.init_node('air_challenge')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

    try:
        print(msg)
        twist = Twist()
        twist.linear.x  = 1.0;
        twist.angular.z = 0.8;
        print(twist)
        pub.publish(twist)
        time.sleep(5)

    except Exception as e:
        print(e)

    finally:
        print("stopping..")
        twist = Twist()
        twist.linear.x = 0;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)


