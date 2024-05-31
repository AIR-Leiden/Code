#!/usr/bin/env python
# coding=utf-8
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from functools import partial
import numpy as np
import sys, select, termios, tty, cv2, rospy

settings = termios.tcgetattr(sys.stdin)

BRIDGE = CvBridge()
TUNNEL_DATA = {
    0: 2.503000020980835,
    10: 2.500999927520752,
    20: 2.513000011444092,
    30: 2.5369999408721924,
    40: 2.552000045776367,
    50: 2.562000036239624,
    60: 2.5850000381469727,
    70: 0.6069999933242798,
    80: 0.5389999747276306,
    90: 0.492000013589859,
    100: 0.40700000524520874,
    110: 0.39100000262260437,
    120: 0.37400001287460327,
    130: 0.32600000500679016,
    140: 0.3140000104904175,
    150: 0.29100000858306885,
    160: 0.2759999930858612,
    170: 0.2460000067949295,
    180: 0.24500000476837158,
    190: 0.23399999737739563,
    200: 0.22499999403953552,
    210: 0.22499999403953552,
    220: 0.21400000154972076,
    230: 0.20600000023841858,
    240: 0.19599999487400055,
    250: 0.19499999284744263,
    260: 0.1850000023841858,
    270: 0.18700000643730164,
    280: 0.17800000309944153,
    290: 0.1809999942779541,
    300: 0.17499999701976776,
    310: 0.16699999570846558,
    320: 0.17000000178813934,
    330: 0.17299999296665192,
    340: 0.16599999368190765,
    350: 0.16899999976158142,
    360: 0.16599999368190765,
    370: 0.17100000381469727,
    380: 0.17299999296665192,
    390: 0.17399999499320984,
    400: 0.17599999904632568,
    410: 0.16699999570846558,
    420: 0.16599999368190765,
    430: 0.16699999570846558,
    440: 0.16699999570846558,
    450: 0.17599999904632568,
    460: 0.17599999904632568,
    470: 0.17399999499320984,
    480: 0.17299999296665192,
    490: 0.17299999296665192,
    500: 0.1720000058412552,
    510: 0.17299999296665192,
    520: 0.17499999701976776,
    530: 0.17100000381469727,
    540: 0.17599999904632568,
    550: 0.17299999296665192,
    560: 0.18199999630451202,
    570: 0.18000000715255737,
    580: 0.18700000643730164,
    590: 0.19699999690055847,
    600: 0.20499999821186066,
    610: 0.20399999618530273,
    620: 0.21400000154972076,
    630: 0.210999995470047,
    640: 0.22200000286102295,
    650: 0.23499999940395355,
    660: 0.23899999260902405,
    670: 0.2630000114440918,
    680: 0.2669999897480011,
    690: 0.2770000100135803,
    700: 0.3009999990463257,
    710: 0.328000009059906,
    720: 0.3499999940395355,
    730: 0.3779999911785126,
    740: 0.44699999690055847,
    750: 0.4950000047683716,
    760: 1.0099999904632568,
    770: 1.0140000581741333,
    780: 0.9819999933242798,
    790: 0.9580000042915344,
    800: 0.925000011920929,
    810: 0.9139999747276306,
    820: 0.8989999890327454,
    830: 0.8759999871253967,
    840: 0.8650000095367432,
    850: 0.8539999723434448,
    860: 0.8399999737739563,
    870: 0.8159999847412109,
    880: 0.8059999942779541,
    890: 0.796999990940094,
    900: 0.7870000004768372,
    910: 0.7870000004768372,
    920: 0.5289999842643738,
    930: 0.4699999988079071,
    940: 0.4090000092983246,
    950: 0.37700000405311584,
    960: 0.36800000071525574,
    970: 0.3499999940395355,
    980: 0.3109999895095825,
    990: 0.2980000078678131,
    1000: 0.28700000047683716,
    1010: 0.25699999928474426,
    1020: 0.25200000405311584,
    1030: 0.23899999260902405,
    1040: 0.23899999260902405,
    1050: 0.22699999809265137,
    1060: 0.22599999606609344,
    1070: 0.21799999475479126,
    1080: 0.20800000429153442,
    1090: 0.20900000631809235,
    1100: 0.20000000298023224,
    1110: 0.19200000166893005,
    1120: 0.1940000057220459,
    1130: 0.18400000035762787,
    1140: 0.1850000023841858,
    1150: 0.1850000023841858,
    1160: 0.1850000023841858,
    1170: 0.17900000512599945,
    1180: 0.17900000512599945,
    1190: 0.18199999630451202,
    1200: 0.1850000023841858,
    1210: 0.18700000643730164,
    1220: 0.18000000715255737,
    1230: 0.18000000715255737,
    1240: 0.18400000035762787,
    1250: 0.1860000044107437,
    1260: 0.17800000309944153,
    1270: 0.17599999904632568,
    1280: 0.17900000512599945,
    1290: 0.18700000643730164,
    1300: 0.18700000643730164,
    1310: 0.1860000044107437,
    1320: 0.1850000023841858,
    1330: 0.1889999955892563,
    1340: 0.18700000643730164,
    1350: 0.18199999630451202,
    1360: 0.1850000023841858,
    1370: 0.1899999976158142,
    1380: 0.1860000044107437,
    1390: 0.19200000166893005,
    1400: 0.18799999356269836,
    1410: 0.1979999989271164,
    1420: 0.19699999690055847,
    1430: 0.20800000429153442,
    1440: 0.210999995470047,
    1450: 0.21199999749660492,
    1460: 0.22300000488758087,
    1470: 0.23000000417232513,
    1480: 0.23199999332427979,
    1490: 0.23999999463558197,
    1500: 0.2619999945163727,
    1510: 0.26600000262260437,
    1520: 0.28700000047683716,
    1530: 0.30000001192092896,
    1540: 0.32199999690055847,
    1550: 0.33899998664855957,
    1560: 0.36899998784065247,
    1570: 0.4099999964237213,
    1580: 0.4390000104904175,
    1590: 0.48899999260902405,
    1600: 0.5289999842643738,
    1610: 0.5839999914169312,
    1620: 2.5239999294281006,
    1630: 2.5160000324249268,
    1640: 2.500999927520752,
    1650: 2.502000093460083,
    1660: 2.490999937057495,
    1670: 2.502000093460083
}

minArea = 400
upper_bound = 100
middle_bound = 250
lower_bound = 400
ROBOT_X = None
biggest_a = 0
LOOP = 0
robot_found = False
DINGO = []


def sign(x):
    if x < 0:
        return -1
    elif x > 0:
        return 1
    else:
        return 0


pass


def lerp(a, b, t):
    return a + t * (b - a)


pass



def hide(scan):
    global LOOP, ROBOT_X, robot_found
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)

    # if true, the robot looks for obstacles that are far away (directly in front of it)
    # far_away_avoidance = False

    # Our robot has a fixed speed
    speed = -0.5  # m/s
    torque = 1.25
    stop = 0.0
    # lidar vars
    data = scan.ranges
    center = data[840:920] + data[760:840]
    # front_right = data[1540:1610]
    # front_left = data[70:140]
    center_right = data[760:840]
    center_left = data[840:920]
    # min_center_right_far = min(data[1640:1680])
    # min_center_left_far = min(data[0:40])
    min_center_right = min(center_right)
    min_center_left = min(center_left)
    min_center = min(center)
    # min_right = min(front_right)
    # min_left = min(front_left)
    twist = Twist()

    R = float("inf")
    L = float("inf")

    # Detect wall in tunnel on left side
    for i in range(80, 410, 10):
        if min(data[i + 840:i + 840 + 10]) < TUNNEL_DATA[i]:
            L = min(data[i + 840:i + 840 + 10])

    # Detect wall in tunnel on right side
    for j in range(1260, 1600, 10):
        if min(data[j - 840:j - 840 + 10]) < TUNNEL_DATA[j]:
            R = min(data[j - 840:j - 840 + 10])

        # Detection in front of robot (general
        if min_center > 0.6:
            twist.linear.x = speed

            # print(sign(ROBOT_X))
            # twist.angular.z = 0.345 * torque * sign(ROBOT_X)
            # ROBOT_X = lerp(ROBOT_X, 0.0, 0.567)
            LOOP = 0
        else:
            twist.linear.x = stop
            LOOP += 1
            twist.angular.z = torque * sign(min_center_left - min_center_right)

        # # Choosing a side to turn to (far away (optional))
        #     if min_center_left2 < 1.5 or min_center_right2 < 1.5:
        #         twist.angular.z = torque * sign(min_center_left_far - min_center_right_far)
        #         loop += 1

        # Choosing a side to turn to (precise)
        if R != float("inf") or L != float("inf"):
            if R != float("inf") and L != float("inf"):
                twist.linear.x = stop
            twist.angular.z = torque * sign(L - R)
            LOOP += 1

    # Counter for getting unstuck
    if LOOP > 100:
        twist.linear.x = -speed
        twist.angular.z = -torque
    pub.publish(twist)


pass

TIME = 0.0


def proc_lidar(scan):
    global DINGO, TIME, LOOP, ROBOT_X, robot_found
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=1)

    # if true, the robot looks for obstacles that are far away (directly in front of it)
    # far_away_avoidance = False

    # Our robot has a fixed speed
    speed = 0.125  # m/s
    torque = 1.25
    stop = 0.0
    # lidar vars
    data = scan.ranges
    center = data[0:80] + data[1600:1680]
    # front_right = data[1540:1610]
    # front_left = data[70:140]
    center_right = data[1600:1680]
    center_left = data[0:80]
    # min_center_right_far = min(data[1640:1680])
    # min_center_left_far = min(data[0:40])
    min_center_right = min(center_right)
    min_center_left = min(center_left)
    min_center = min(center)
    # min_right = min(front_right)
    # min_left = min(front_left)
    twist = Twist()

    R = float("inf")
    L = float("inf")

    # Detect wall in tunnel on left side
    for i in range(80, 410, 10):
        if min(data[i:i + 10]) < TUNNEL_DATA[i]:
            L = min(data[i:i + 10])

    # Detect wall in tunnel on right side
    for j in range(1260, 1600, 10):
        if min(data[j:j + 10]) < TUNNEL_DATA[j]:
            R = min(data[j:j + 10])

    DINGO.append(1 if ROBOT_X else 0)
    # Detection in front of robot (general)
    if TIME > 0.0:
        print(TIME)
        TIME -= 1.0
        return hide(scan)
    if ROBOT_X is not None and ((-300.0 < ROBOT_X and ROBOT_X < -100.0) or (100.0 < ROBOT_X and ROBOT_X < 300.0)):
        if len(DINGO) > 25:
            print(DINGO)
            if sum(DINGO) / len(DINGO) > 0.5:
                TIME = 100.0
                return hide(scan)
            DINGO = []
        # twist.linear.x = 0.1 * speed
        # twist.angular.z = -0.375 * torque * sign(ROBOT_X)
    else:
        if min_center > 0.5:
            twist.linear.x = speed

            # print(sign(ROBOT_X))
            # twist.angular.z = 0.345 * torque * sign(ROBOT_X)
            # ROBOT_X = lerp(ROBOT_X, 0.0, 0.567)
            LOOP = 0
        else:
            twist.linear.x = stop
            LOOP += 1
            twist.angular.z = torque * sign(min_center_left - min_center_right)

        # # Choosing a side to turn to (far away (optional))
        #     if min_center_left2 < 1.5 or min_center_right2 < 1.5:
        #         twist.angular.z = torque * sign(min_center_left_far - min_center_right_far)
        #         loop += 1

        # Choosing a side to turn to (precise)
        if R != float("inf") or L != float("inf"):
            if R != float("inf") and L != float("inf"):
                twist.linear.x = stop
            twist.angular.z = torque * sign(L - R)
            LOOP += 1

        # Counter for getting unstuck
    if LOOP > 100:
        twist.linear.x = -speed
        twist.angular.z = -torque
    pub.publish(twist)


pass


def proc_camera(img):
    def area(contour, color_name, color_BGR, color_img):
        global ROBOT_X, robot_found, biggest_a
        a = cv2.contourArea(contour)
        if a > minArea:
            x, y, w, h = cv2.boundingRect(contour)
            if (w / h < 2) or ((y + (h / 2)) < upper_bound) or ((y + (h / 2)) > lower_bound) or (
                    (y + (h / 2) > middle_bound) and (a < 10000)):
                return
            # color_img = cv2.rectangle(color_img, (x, y), (x + w, y + h), color_BGR, 2)
            #cv2.putText(color_img, color_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_BGR)
            if a > biggest_a:
                biggest_a = a
                ROBOT_X = (x + 0.5 * w) - 320.0
            if a > 30000:
                robot_found = True
                print("Robot_found")
                # rospy.signal_shutdown("Found")

    global ROBOT_X
    global biggest_a
    ROBOT_X = None
    biggest_a = 0
    cv_img = BRIDGE.imgmsg_to_cv2(img, "passthrough")
    color_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    rgb_frame = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

    red_lower = np.array([95, 24, 19], np.uint8)
    red_upper = np.array([165, 46, 84], np.uint8)
    red_mask = cv2.inRange(rgb_frame, red_lower, red_upper)

    purple_lower = np.array([40, 29, 124], np.uint8)
    purple_upper = np.array([114, 108, 195], np.uint8)
    purple_mask = cv2.inRange(rgb_frame, purple_lower, purple_upper)


    kernel = np.ones((5, 5), "uint8")

    red_mask = cv2.dilate(red_mask, kernel)

    purple_mask = cv2.dilate(purple_mask, kernel)

    contours = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

    for pic, contour in enumerate(contours):
        area(contour, "Red", (0, 0, 255), color_img)

    contours = cv2.findContours(purple_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

    for pic, contour in enumerate(contours):
        area(contour, "Purple", (140, 0, 140), color_img)


    # cv2.line(color_img, (0, upper_bound), (640, upper_bound), (0, 0, 0), 4)
    # cv2.line(color_img, (0, middle_bound), (640, middle_bound), (0, 0, 0), 4)
    # cv2.line(color_img, (0, lower_bound), (640, lower_bound), (0, 0, 0), 4)
    # cv2.imshow("Camera", color_img)
    cv2.waitKey(1)
    print(ROBOT_X, biggest_a)


pass

LIDAR = 0
CAMERA = 1


def ros_main(node, data):
    # Check which node, then call designated callback
    if LIDAR == node:
        proc_lidar(data)
    elif CAMERA == node:
        proc_camera(data)


pass


def ros_exit():
    # Stop motors
    pub = rospy.Publisher("~cmd_vel", Twist, queue_size=1)
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)

    # Stop cv2
    cv2.destroyAllWindows()

    sys.exit()


pass

if __name__ == "__main__":
    rospy.init_node("rosberrypie",
                    anonymous=True,
                    log_level=rospy.INFO,
                    disable_signals=False)
    rospy.on_shutdown(ros_exit)

    rospy.Subscriber("/scan", LaserScan, partial(ros_main, LIDAR))
    rospy.Subscriber("/camera/rgb/image_raw", Image, partial(ros_main, CAMERA))

    # Keep threads running (same as while True, with exception handling and rospy.sleep())

    rospy.spin()
pass
