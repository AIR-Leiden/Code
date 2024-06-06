#!/usr/bin/env python
# coding=utf-8
"""
We will move your robot!

Make sure the robot is on the ground, there are no cables around and that there is nothing in front
of the robot!
"""

# 1. add pipe for planning request: list of goals or None to explore


import gc
import multiprocessing
import threading
import traceback
from datetime import datetime

import genpy
import std_msgs
from tf.transformations import euler_from_quaternion, euler_from_matrix
from typing import Optional, List

import numpy as np
from tf2_geometry_msgs import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
import tf

from getfrontier import getfrontier
import rospy
import time
from nav_msgs.msg import MapMetaData, Odometry, OccupancyGrid
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal

from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3Stamped, TransformStamped, \
    PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan, Image

from coordinates_util import *
from utils import *

import sys, select, cv2

from astar import astar, find_position, find_all_positions
from cluster_detection import lidar_callback, get_other_robot
from planning import planner

print(rospy.__file__)
print(len(sys.path))
import os

# before using this script make sure that you run
# roslaunch turn_on_wheeltec_robot mapping.launch
# roslaunch turn_on_wheeltec_robot wheeltec_camera.launch camera_mode:=Astra_Pro+RgbCam
# in separate terminals

import signal

ODOM_TOPIC = "/robot_pose_ekf/odom_combined"
ODOM_FRAME_ID = "odom_combined"

print "pid: " + str(multiprocessing.current_process().pid)

stop = False

def handler(signum, frame):
    global stop
    print("SIGINT, pid: " + str(multiprocessing.current_process().pid))
    if planning_process is not None:
        planning_process.terminate()
    exit(-1)

    stop = True



signal.signal(signal.SIGINT, handler)

other_robot_position_camera = None


def group3_vrd_callback(msg):
    # type: (std_msgs.msg.String) -> None
    global other_robot_position_camera

    print "group3_vrd_callback"

    other_robot_position_camera = \
        [
            float(entry)
            for entry in msg.data.split(",")
        ]

    other_robot_position_camera[2] = time.time()


def get_other_robot_camera():
    global other_robot_position_camera
    return other_robot_position_camera


occupancy_grid = None  # type: Optional[OccupancyGrid]


def map_callback(data):
    # type: (OccupancyGrid) -> None
    global occupancy_grid
    clear_queu_and_send(
        planning_map_queue,
        data
    )
    occupancy_grid = data


odom = None  # type: Optional[PoseWithCovarianceStamped]
tf_message = None  # type: Optional[TFMessage]
tf_listener = None  # type: Optional[tf.TransformListener]

def robot_pose():
    # type: () -> Pose
    """
    :return: the robots pose in the world
    """
    global odom

    pose_stamped = PoseStamped()
    pose_stamped.pose = odom.pose.pose
    pose_stamped.header = odom.header
    pose_stamped.header.stamp = tf_listener.getLatestCommonTime(pose_stamped.header.frame_id, "map")

    out = tf_listener.transformPose("map", pose_stamped).pose


    return out


def euler_angle_from_pose(pose):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return euler_from_quaternion(q)

last_robot_pose = None
last_velocity_update = 0
approx_x_velocity = 0
approx_y_velocity = 0
approx_angular_z_velocity = 0

def update_velocity():
    global last_robot_pose, last_velocity_update, approx_y_velocity, approx_x_velocity
    pose = robot_pose()

    if pose == last_robot_pose:
        return

    print(euler_angle_from_pose(pose))

    t = time.time()
    dt = t - last_velocity_update
    if last_robot_pose != None:
        approx_x_velocity = (pose.position.x - last_robot_pose.position.x) / dt
        approx_y_velocity = (pose.position.y - last_robot_pose.position.x) / dt
        approx_angular_z_velocity = (euler_angle_from_pose(pose)[2] - euler_angle_from_pose(last_robot_pose)[2]) / dt
        print "approx:", approx_angular_z_velocity

    last_robot_pose = pose
    last_velocity_update = t

def robot_position():
    # type: () -> np.ndarray
    """
    :return: the robots position in real-world coordinates
    """
    pose = robot_pose()

    return np.array([
        pose.position.x,
        pose.position.y,
    ])


def grid_origin():
    # type: () -> np.ndarray
    global occupancy_grid
    return np.array([
        occupancy_grid.info.origin.position.x,
        occupancy_grid.info.origin.position.y,
    ])


def realworld_to_grid(position):
    # type: (np.ndarray) -> np.ndarray
    global occupancy_grid
    assert position.shape == (2,)
    return ((position - grid_origin()) / occupancy_grid.info.resolution).astype(int)


def grid_to_realworld(position):
    # type: (np.ndarray) -> np.ndarray
    global occupancy_grid
    assert position.shape == (2,)
    return position.astype(float) * occupancy_grid.info.resolution + grid_origin()


def odom_callback(data):
    global odom
    odom = data

    if not did_start:
        return

    clear_queu_and_send(
        planning_grid_position_queue,
        realworld_to_grid(robot_position())
    )


def tf_callback(data):
    # type: (TFMessage) -> None
    global tf_message
    tf_message = data

    if not did_start:
        return

    clear_queu_and_send(
        planning_grid_position_queue,
        realworld_to_grid(robot_position())
    )


def driving_vector_to_twist_command(driving_vector):
    # type: (np.ndarray) -> Twist
    """
    driving vector is measured in m/s
    :param driving_vector:
    :return:
    """

    v = Vector3Stamped()
    v.vector.x = driving_vector[0]
    v.vector.y = driving_vector[1]
    t = TransformStamped()
    t.transform.rotation.x = robot_pose().orientation.x
    t.transform.rotation.y = robot_pose().orientation.y
    t.transform.rotation.z = robot_pose().orientation.z
    t.transform.rotation.w = -robot_pose().orientation.w

    vt = tf2_geometry_msgs.do_transform_vector3(v, t)

    twist = Twist()
    twist.linear.x = vt.vector.x
    twist.linear.y = vt.vector.y
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    z_set_speed = 0.5 # 0.5 if (vt.vector.x ** 2 + vt.vector.y ** 2) < (0.3 ** 2) else 0

    other_robot = get_other_robot_camera()
    if is_valid_other_robot(other_robot):
        dx, dy, t, angle = other_robot

        z_set_speed = - angle * 0.05

        print "DRIVING TOWARD OTHER ROBOT", angle

    twist.angular.z = z_set_speed # * 2 - approx_angular_z_velocity
    return twist


def is_valid_other_robot(other_robot):
    if other_robot is None:
        return False
    return time.time() - other_robot[2] < 0.3


def calculate_driving_vector(target_position):
    if target_position is None:
        return point(0, 0)
    position = realworld_to_grid(robot_position())

    direction = target_position - position  # type: np.ndarray

    direction_len = np.sqrt(np.sum(direction ** 2))
    if direction_len < 0.01:
        return point(0, 0)

    max_speed = 0.7
    speed_p = 0.15

    return (direction / direction_len) * min(direction_len * speed_p, max_speed)


# this is the function that actually decides to which point we want to drive right now
def send_planning_request():
    # print "send_planning_request()"
    planning_request = None

    other_robot = get_other_robot_camera()
    if is_valid_other_robot(other_robot):
        # TODO: this code is not correct right now:
        # dx, dy are not in the same coordinate system as x and y,
        # this is because dx and dy are relative to the camera. the vectory (dx, dy) needs to be rotated by
        # the opposite of the robots rotation in the world. This can probably be done by a quaternion rotation
        # similar to the one in driving_vector_to_twist_command, as the driving direction has a similar problem:
        # it needs the same rotation but from world coordinates to robot coordinates.


        x, y = robot_position()
        dx_untransformed, dy_untransformed, t, angle = other_robot  # assume that dx, dy is 1m for now...

        v = Vector3Stamped()
        v.vector.x = dx_untransformed
        v.vector.y = dy_untransformed
        t = TransformStamped()
        t.transform.rotation.x = robot_pose().orientation.x
        t.transform.rotation.y = robot_pose().orientation.y
        t.transform.rotation.z = robot_pose().orientation.z
        t.transform.rotation.w = robot_pose().orientation.w

        vt = tf2_geometry_msgs.do_transform_vector3(v, t)

        dx = vt.vector.x
        dy = vt.vector.y


        planning_request = [
            tuple(point_int(*realworld_to_grid(point(
                x + dx * distance,
                y + dy * distance
            ))))
            for distance in [0.5]
        ]
        print realworld_to_grid(robot_position()), planning_request

    clear_queu_and_send(
        planning_request_queue,
        planning_request
    )


path = None  # type: Optional[List[np.ndarray]]
recent_spots = []
target_position = None


def calculate_target_pos():
    global target_position
    target_position = None
    if path is not None:
        to_remove = []
        for i in range(len(path)):
            t = path[i]
            near_recent = False
            for recent in recent_spots:
                if ((t - recent) ** 2).sum() < 6 ** 2:
                    near_recent = True
                    break
            if not near_recent:
                target_position = t
                break
            else:
                to_remove.append(i)
        for i in sorted(to_remove, reverse=True):
            del path[i]

did_start = False
if __name__ == "__main__":
    planning_request_queue = multiprocessing.Queue()
    planning_map_queue = multiprocessing.Queue()
    planning_grid_position_queue = multiprocessing.Queue()
    planning_path_queue = multiprocessing.Queue()
    planning_process = multiprocessing.Process(
        target=planner,
        args=(
            planning_request_queue,
            planning_map_queue,
            planning_grid_position_queue,
            planning_path_queue
        )
    )
    print("starting planning process...")
    planning_process.start()
    print("done..")

    rospy.init_node('air_challenge', disable_signals=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size=2)
    rospy.Subscriber(ODOM_TOPIC, PoseWithCovarianceStamped, odom_callback, queue_size=2)
    rospy.Subscriber("/tf", TFMessage, tf_callback, queue_size=2)
    rospy.Subscriber("/group3_vrd", std_msgs.msg.String, group3_vrd_callback, queue_size=1)

    tf_listener = tf.TransformListener()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0, tcp_nodelay=True, latch=True)

    start_time = datetime.now()
    print("Starting at {}".format(start_time))

    while not stop and (occupancy_grid is None or odom is None or tf_message is None):
        print("waiting for occupancy and odom data")
        print odom is None
        print occupancy_grid is None
        print tf_message is None
        time.sleep(0.1)

    time.sleep(0.1)
    did_start = True

    loop = 0
    r = rospy.Rate(100)
    last_loop_time = time.time()
    while not stop:
        loop += 1
        try:
            update_velocity()
            send_planning_request()

            pos = list(realworld_to_grid(robot_position()))
            if not recent_spots.__contains__(pos):
                recent_spots.append(pos)
            while len(recent_spots) > 7:
                del recent_spots[0]

            while not planning_path_queue.empty() > 0:
                path = planning_path_queue.get() or path

            calculate_target_pos()

            driving_vector = calculate_driving_vector(target_position)

            twist = driving_vector_to_twist_command(driving_vector)
            print("Z speed:", twist.angular.z)
            pub.publish(twist)

            # gc.collect(0)

            if time.time() - last_loop_time > 0.15:
                print "main loop took too long!"
            last_loop_time = time.time()

            r.sleep()

        except:
            stop = True
            traceback.print_exc()
            print "stopping main loop due to exception"

    stop = True

    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    print "sending stop"
    time.sleep(0.1)
    rospy.signal_shutdown("stopped by python")
    print "1"
    planning_process.terminate()
    print "2"
    planning_process.join()
    print "3"
    os.kill(os.getpid(), signal.SIGTERM)

