#!/usr/bin/env python
import time
import os
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import cv2

from math import sin, cos, pi, isinf

# GLOBALS
pub = None


def idx_to_rad(idx):
    return pi * (idx / 840.0)


def set_zero_speed():
    global pub
    speed = Twist()
    speed.angular.z = 0
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    pub.publish(speed)


def distance_from_robot(range_idx, raw_range):
    robot_half_length = 0.22
    robot_half_width = 0.16
    if isinf(raw_range):
        return raw_range
    angle = ((range_idx / 1681.0) * (2 * pi))
    # Front quadrant
    if (range_idx < 210) or (range_idx > 1471):
        robot_size_at_angle = (1 / (cos(angle) / robot_half_width))
        return raw_range - robot_size_at_angle
    # Back
    elif 686 < range_idx < 994:
        robot_size_at_angle = (1 / (cos(angle - (2 * pi)) / robot_half_length))
        return raw_range - (-1 * robot_size_at_angle)
    # Sides
    else:
        robot_size_at_angle = (1 / abs(sin(angle) / robot_half_width))
        return raw_range - robot_size_at_angle


def callback_laser(msg):
    # ____ msg.data length is 1681, thus 1 index is 360/1681 = 0.214 deg = 0.0037 rad ;___
    #
    # 15deg = 70, 30deg = 140, 45deg = 210, 60deg = 280, ! 90deg = 420 !, ! 180deg = 840 !, ! 270deg = 1261 ! ;
    # -15deg = 1611, -30deg = 1541, -45deg = 1471, -60deg = 1401, -90deg = 1261 ;

    global min_distance
    global min_distance_index
    global pub

    ranges_real = [distance_from_robot(i, x) for i, x in enumerate(msg.ranges)]

    min_distance = min(ranges_real)
    min_distance_index = ranges_real.index(min_distance)

    speed = min(0.3, (0.075 / (min_distance)**2))

    velocity_idx = min_distance_index + 840
    if velocity_idx > 1681:
        velocity_idx -= 1681
    velocity_ang = idx_to_rad(velocity_idx)

    x_part = cos(velocity_ang)
    y_part = sin(velocity_ang)

    outmsg = Twist()
    outmsg.linear.x = (x_part * speed)
    outmsg.linear.y = (y_part * speed)

    pub.publish(outmsg)
    print("PUBlishing.")


def callback_cam(img):      # this is just here so we can watch
    global pos_folder
    global neg_folder
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, 'bgr')

    cv2.imshow('Camera', cv_image)

    loop_time = time.time()

    key = cv2.waitKey(1)
    if key == ord('q'):
        set_zero_speed()
        rospy.signal_shutdown("User interrupt.")
    


if __name__ == "__main__":
    rospy.init_node('runner', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback_cam)
    rospy.Subscriber('/scan', LaserScan, callback_laser)
    rospy.spin()
    cv2.destroyAllWindows()
