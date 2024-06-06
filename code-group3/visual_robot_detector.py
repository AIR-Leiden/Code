#!/usr/bin/env python
# coding=utf-8
import math
import sys

import rospy

import numpy as np

import cv2

import time

import std_msgs
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from utils import *

bridge = CvBridge()

other_robot_position_camera = None

DEBUG_VIEW = len(sys.argv) > 1 and sys.argv[1] == "DEBUG"

def hex_to_hsv(hex_rgb):
    # Convert hexadecimal RGB to BGR
    hex_rgb = hex_rgb.strip("#")
    bgr = np.array([int(hex_rgb[i:i+2], 16) for i in (0, 2, 4)][::-1])
    # Reshape the array to a 1x1x3 matrix
    bgr = bgr.reshape(1, 1, 3).astype(np.uint8)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return hsv[0, 0]


kernel_size = 4
kernel = np.ones((kernel_size, kernel_size), np.float32) / (kernel_size**2)

def camera_callback(msg):
    # type: (Image) -> None
    global other_robot_position_camera

    print "=== GOT CAMERA FRAME"

    start = time.time()

    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
    frame = frame[
            int(frame.shape[0] * 0.5):,
            : # int(frame.shape[1] * 0.5): # int(frame.shape[1] * 0.8)
            ]

    frame = cv2.resize(frame, (frame.shape[1]/2, frame.shape[0]/2), interpolation= cv2.INTER_LINEAR)
    frame = cv2.filter2D(frame, -1, kernel)

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    if DEBUG_VIEW:
        cv2.imshow('image', cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        cv2.waitKey(0)

    # orange: 17, 62, 84
    # yellow: 38, 50, 75

    #h = 17 * 255 / 360  # 17
    #s = 62 * 255 / 100  # 62
    #v = 91 * 255 / 100  # 84

    colors = [
        hex_to_hsv("#992a0d"), # orange robot, new building
        hex_to_hsv("#cd7756"),
        hex_to_hsv("#1b46b6"), # blue robot new building
        hex_to_hsv("#b83d4c"), # "red robot new building
        hex_to_hsv("#c9a070"), # yellow robot new building
        hex_to_hsv("#37397c"), # blue robot new building
    ]

    allPoints = np.zeros((0, 1, 2), dtype=np.int)
    for color in colors:
        h, s, v = color

        s_tolerance = 3
        h_tolerance = 3
        v_tolerance = 15

        lower_red = np.array([
            max(h - h_tolerance, 0),
            max(s - s_tolerance, 0),
            max(v - v_tolerance, 10)
        ])
        upper_red = np.array([
            min(h + h_tolerance, 255),
            min(s + s_tolerance, 255),
            min(v + v_tolerance, 255)
        ])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        points = cv2.findNonZero(mask)

        if points is not None:
            allPoints = np.concatenate((allPoints, points))

    if len(allPoints) == 0:
        return None

    avg = np.median(allPoints, axis=0)


    camera_fov_degrees = 60
    angle = \
        ((avg[0][0] / float(frame.shape[1])) - 0.5) * camera_fov_degrees

    end = time.time()
    print angle, avg[0][0], frame.shape, end - start

    dx, dy = calculate_other_robot_position(angle, 1)

    other_robot_position_camera = (dx, dy, time.time(), angle)

    print "sending.. to " + str(pub.get_num_connections()) + " subscribers"

    pub.publish(std_msgs.msg.String(
        ",".join([
            str(entry)
            for entry in other_robot_position_camera
        ])
    ))

    print "sent!"



def get_other_robot_camera():
    global other_robot_position_camera
    return other_robot_position_camera


def calculate_other_robot_position(other_robot_angle, other_robot_distance):
    dx = math.cos(math.radians(other_robot_angle)) * other_robot_distance
    dy = math.sin(math.radians(other_robot_angle)) * other_robot_distance
    return dx, dy


if __name__ == "__main__":
    rospy.init_node('air_challenge_2_camera')
    rospy.Subscriber("/camera/rgb/image_raw", Image, camera_callback)
    pub = rospy.Publisher('group3_vrd', std_msgs.msg.String, queue_size=100)
    rospy.spin()
