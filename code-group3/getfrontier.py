#!/usr/bin/env python


# --------Include modules---------------
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2


# -----------------------------------------------------

def getfrontier(mapData, data_2d):
    data = mapData.data
    w = mapData.info.width
    h = mapData.info.height
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    # Define the conditions and corresponding values
    conditions = [data_2d == 100, data_2d == 0, data_2d == -1]
    values = [0, 255, 205]

    # Use np.select to assign values based on conditions
    img = (np.select(conditions, values, default=0)
           .reshape((h, w, 1))
           .astype(np.uint8))

    o = cv2.inRange(img, 0, 1)
    edges = cv2.Canny(img, 0, 255)
    im2, contours, hierarchy = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(o, contours, -1, (255, 255, 255), 5)
    o = cv2.bitwise_not(o)
    res = cv2.bitwise_and(o, edges)
    # ------------------------------

    frontier = copy(res)
    im2, contours, hierarchy = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frontier, contours, -1, (255, 255, 255), 2)

    im2, contours, hierarchy = cv2.findContours(frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    all_pts = np.zeros((0,0))
    if len(contours) > 0:
        upto = len(contours) - 1
        i = 0
        maxx = 0
        maxind = 0

        for i in range(0, len(contours)):
            cnt = contours[i]
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            xr = cx * resolution + Xstartx
            yr = cy * resolution + Xstarty
            pt = np.array([[cx, cy]])
            if len(all_pts) > 0:
                all_pts = np.vstack([all_pts, pt])
            else:
                all_pts = pt

    return [(point[0], point[1]) for point in all_pts]

