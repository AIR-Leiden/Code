#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
import cv2
import numpy as np
import time

class RunAway:
    def __init__(self, color_lower, color_upper):
        rospy.init_node('get_him_node', anonymous=True)

        self.image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.obstacle_threshold = 1.1
        self.rear_scan_range = (-15, 15)
        self.rear_scan_range_turning = (-60, 60)
        self.linear_speed = 0.3
        self.angular_speed = 3.5
        # self.linear_speed = 1
        # self.angular_speed = 10

        self.bridge = CvBridge()
        self.color_lower = np.array(color_lower)
        self.color_upper = np.array(color_upper)
        self.cv_image = None

        self.detected_robot = False
        self.line_height_percentage = 30

        rospy.on_shutdown(self.stop_robot)



    def scan_callback(self, scan_data):
        rear_scan_data = scan_data.ranges[self.degrees_to_index(self.rear_scan_range[0], 'forward'):self.degrees_to_index(self.rear_scan_range[1], 'forward')]
        rear_scan_data_turning = scan_data.ranges[self.degrees_to_index(self.rear_scan_range_turning[0], 'turning'):self.degrees_to_index(self.rear_scan_range_turning[1], 'turning')]
        min_distance = min(rear_scan_data)

        if not self.detected_robot:
            if min_distance < self.obstacle_threshold:
                if min_distance < 0.5:
                    self.avoid_close_obstacle(rear_scan_data_turning)
                else:
                    self.avoid_obstacle(min_distance, rear_scan_data_turning)
            else:
                self.move_forward()



    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        
        line_height = self.cv_image.shape[0] * (100 - self.line_height_percentage) / 100

        self.process_image(line_height)
        cv2.imshow('Original Image', self.cv_image)
        cv2.waitKey(1)



    def apply_filters(self, image):
        # Filters
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        hsv_image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.color_lower, self.color_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((12, 12), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((12, 12), np.uint8))
        _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Filter shapes
        valid_contours = [cnt for i, cnt in enumerate(contours) if cv2.contourArea(cnt) > 100 and hierarchy[0][i][3] == -1]
        valid_contours = [cnt for cnt in valid_contours if self.is_horizontal_shape(cnt)]

        return valid_contours



    def process_image(self, line_height):
        valid_contours = self.apply_filters(self.cv_image)

        # Draw contours and red line
        cv2.drawContours(self.cv_image, valid_contours, -1, (0, 255, 0), 2)
        cv2.line(self.cv_image, (0, line_height), (self.cv_image.shape[1], line_height), (0, 0, 255), 2)

        color_surface_present = len(valid_contours) > 0

        # Check if contour is present
        if color_surface_present:
            M = cv2.moments(valid_contours[0])
            self.detected_robot = True
        
            # Centroid detected
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Move to target or stop
                if cy > line_height:
                    twist_msg = self.stop_robot()
                    print("boop")
                else:
                    twist_msg = self.move_to_target(cx)
                
                self.cmd_vel_pub.publish(twist_msg)

                # Draw centroid
                cv2.circle(self.cv_image, (cx, cy), 7, (255, 255, 255), -1)
        
        else: 
            self.detected_robot = False


    
    def move_to_target(self, cx):
        image_center_x = self.cv_image.shape[1] / 2
        diff = cx - image_center_x
        max_angular_vel = 0.5
        damping_factor = 0.11

        angular_vel = np.clip(-0.01 * diff * damping_factor, -max_angular_vel, max_angular_vel)

        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        twist_msg.angular.z = angular_vel

        return twist_msg



    def is_horizontal_shape(self, contour):
        _, _, w, h = cv2.boundingRect(contour)
        return w > 2 * h



    def avoid_obstacle(self, min_distance, rear_scan_data):
        error = self.obstacle_threshold - min_distance
        angular_velocity = self.angular_speed * error / self.obstacle_threshold

        left_scan_data = rear_scan_data[:len(rear_scan_data)//2]
        right_scan_data = rear_scan_data[len(rear_scan_data)//2:]
        
        min_left_distance = np.mean(left_scan_data)
        min_right_distance = np.mean(right_scan_data)
        
        if min_left_distance > min_right_distance:
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = -angular_velocity
            self.cmd_vel_pub.publish(twist_msg)
        else:
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist_msg)



    def avoid_close_obstacle(self, rear_scan_data):
        left_scan_data = rear_scan_data[:len(rear_scan_data)//2]
        right_scan_data = rear_scan_data[len(rear_scan_data)//2:]
        
        min_left_distance = np.mean(left_scan_data)
        min_right_distance = np.mean(right_scan_data)
        
        if min_left_distance > min_right_distance:
            self.turn_left_on_spot()
        else:
            self.turn_right_on_spot()



    def turn_left_on_spot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = -self.angular_speed
        self.cmd_vel_pub.publish(twist_msg)



    def turn_right_on_spot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist_msg)



    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)



    def degrees_to_index(self, degrees, direction):
        if direction == 'forward':
            angle_min = -15
        elif direction == 'turning':
            angle_min = -60
        angle_increment = 0.25  # Assuming your laser scan has an increment of 0.25 degrees
        return int((degrees - angle_min) / angle_increment)



    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        return twist_msg



    def run(self):
        rospy.spin()



if __name__ == '__main__':
    try:
        color_lower = [0, 100, 100]
        color_upper = [10, 255, 255]

        color = input("Red: 1, Orange: 2, Yellow: 3, Blue: 4, Purple: 5\n")
        if color == 1: # Red
            # color_lower = [0, 90, 90]
            # color_upper = [10, 255, 255]
            color_lower = [150, 50, 50]
            color_upper = [360, 255, 255]
        elif color == 2: # Orange
            color_lower = [5, 100, 100]
            color_upper = [15, 255, 255]
        elif color == 3: # Yellow
            color_lower = [10, 100, 100]
            color_upper = [20, 255, 255]
        elif color == 4: # Blue
            color_lower = [100, 100, 100]
            color_upper = [150, 255, 255]
        elif color == 5: # Purple
            color_lower = [100, 100, 100]
            color_upper = [200, 255, 255]
       
        run_away = RunAway(color_lower=color_lower, color_upper=color_upper)
        run_away.run()

    except rospy.ROSInterruptException:
        pass
