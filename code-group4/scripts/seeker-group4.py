#!/usr/bin/env python
import time
import os
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import cv2

from math import sin, cos, pi, isinf





# GLOBALS
pub = None
pos_folder = "zzz_pos"
neg_folder = "zzz_neg"
seeking = False
center_x = -1


def distance_from_robot(range_idx, raw_range):
    robot_half_length = 0.24
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
    global seeking
    global center_x

    ranges_real = [distance_from_robot(i, x) for i, x in enumerate(msg.ranges)]

    min_distance = min(ranges_real)
    min_distance_index = ranges_real.index(min_distance)

    # print("Smallest real distance: ", min_distance)
    # print("At index: ", min_distance_index)

    outmsg = Twist()

    if seeking and center_x != -1:
        outmsg.linear.x = 0
        outmsg.angular.z = 0
        print(center_x)
        if center_x >= 340:
            print("Going right")
            if center_x >= 400:
                outmsg.linear.x = 0.2
                outmsg.angular.z = -0.3
            else:
                outmsg.linear.x = 0.5
                outmsg.angular.z = -0.3
        elif center_x < 300:
            print("Going left")
            if center_x < 240:
                outmsg.linear.x = 0.2
                outmsg.angular.z = 0.3
            else:
                outmsg.linear.x = 0.5
                outmsg.angular.z = 0.3
        if 240 < center_x < 400:
            outmsg.linear.x = 0.2
            if 300 < center_x < 340:
                outmsg.linear.x = 0.8
    else:   # Wandering
        if min_distance > 0.1:
            outmsg.linear.x = 0.1
        if min_distance > 0.15:
            outmsg.linear.x = 0.3
        if min_distance > 0.4:
            outmsg.linear.x = 0.6
        else:
            outmsg.linear.x = 0
            if min_distance_index > 1260:            # RIGHT 90deg
                outmsg.angular.z = 0.5
            elif min_distance_index < 420:           # LEFT 90deg
                outmsg.angular.z = -0.5
            else:
                outmsg.angular.z = 0
                outmsg.linear.x = 0.3

    pub.publish(outmsg)


def set_zero_speed():
    global pub
    speed = Twist()
    speed.angular.z = 0
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    pub.publish(speed)


def callback_cam(img):
    global pos_folder
    global neg_folder
    global seeking
    global center_x
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    center_x = orange_detection(cv_image)

    if center_x == -1:
        seeking = False
    else:
        seeking = True

    key = cv2.waitKey(1)
    if key == ord('q'):
        set_zero_speed()
        rospy.signal_shutdown("User interrupt.")


def orange_detection(image, show=True, color="red"):
    ''' Returns the center x coordinate of an orange robot if there is one.
    
    Edge values'''

    # Edge values ORANGE
    # hue min = 110                 # detection is very sensitive to hue min
    # hue max = 120                 # hue values are by far the most influential
    # sat min = 90
    # sat max = 255
    # val min = 140
    # val max = 255
    orange = [110, 120, 90, 255, 100, 255]

    # Edge values RED
    # hue min = 117
    # hue max = 124
    # sat min = 95
    # sat max = 255
    # val min = 0
    # val max = 255
    red = [117, 124, 95, 255, 0, 255]

    # Edge values PURPLE
    # hue min = 165
    # hue max = 180
    # sat min = 66
    # sat max = 165
    # val min = 0
    # val max = 255
    purple = [165, 180, 66, 165, 0, 255]

    if color == "red":
        hue_min, hue_max, sat_min, sat_max, val_min, val_max = red
    elif color == "purple":
        hue_min, hue_max, sat_min, sat_max, val_min, val_max = purple
    else:
        hue_min, hue_max, sat_min, sat_max, val_min, val_max = orange

    img = image.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    output = img.copy()

    # Getting only pixels in range
    thresh = cv2.inRange(hsv, (hue_min, sat_min, val_min), (hue_max, sat_max, val_max))

    # Removing noise by erosion
    kernel = np.ones((5, 5))
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts) > 0:
        biggest_c = max(cnts, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(biggest_c)
        centroid = (x + int(w / 2), y + int(h / 2))

        if w > 30:
            cv2.rectangle(output, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 0), 1)
            cv2.circle(output, centroid, 2, (0, 0, 255), 3)

            if show:
                cv2.imshow('Mask', mask)
                cv2.imshow("Image", output)

            return centroid[0]
        else:
            if show:
                cv2.imshow('Mask', mask)
                cv2.imshow("Image", output)
            return -1
    else:
        if show:
            cv2.imshow('Mask', mask)
            cv2.imshow("Image", output)
        return -1
    



if __name__ == "__main__":
    rospy.init_node('color_detection', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback_cam)
    rospy.Subscriber('/scan', LaserScan, callback_laser)
    rospy.spin()
    set_zero_speed()
    cv2.destroyAllWindows()
