#!/usr/bin/env python
# coding=utf-8

from datetime import datetime

import rospy
from geometry_msgs.msg import Twist

msg = """
We will move your robot!

Make sure there is nothing in front of the robot.
"""

def check_stop(start_time, seconds=2):
    """
    Check if the robot should stop depending on the time elapsed in milliseconds (ms)
    """
    now = datetime.now()
    delta = now - start_time
    return (delta.seconds) > seconds


if __name__=="__main__":

    rospy.init_node('air_challenge')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)

    speed = 0.2 # m/s
    turn = 0.5 # rad/s

    start_time = datetime.now()
    print("Starting at {}".format(start_time))
    try:
        while not check_stop(start_time):
            print("  Moving forward!")
            twist = Twist()
            twist.linear.x  = speed;
            # twist.angular.z = turn;
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        print("Stopping..")
        twist = Twist()
        twist.linear.x = 0;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)


