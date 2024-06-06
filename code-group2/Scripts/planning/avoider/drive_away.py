#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64, String
import cv2
import numpy as np
import time

class RunAway:
    def __init__(self, color_lower, color_upper):
        rospy.init_node('get_him_node', anonymous=True)

        self.image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.pause_pub = rospy.Publisher('/pause_topic', String, queue_size=10)
        self.linear_speed = 0.3
        self.angular_speed = 3.5

        self.bridge = CvBridge()
        self.color_lower = np.array(color_lower)
        self.color_upper = np.array(color_upper)
        self.cv_image = None

        self.detected_robot = False
        self.line_height_percentage = 30



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
        pause_msg = String()
        pause_msg.data = 'resume'
        self.pause_pub.publish(pause_msg)
        if color_surface_present:
            M = cv2.moments(valid_contours[0])
            self.detected_robot = True
        
            # Centroid detected
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Move to target or stop
                if cy > line_height:
                    print("boop")
                else:
                    pause_msg = String()
                    pause_msg.data = 'pause'
                    self.pause_pub.publish(pause_msg)
                

                # Draw centroid
                cv2.circle(self.cv_image, (cx, cy), 7, (255, 255, 255), -1)
        
        else: 
            self.detected_robot = False


    


    def is_horizontal_shape(self, contour):
        _, _, w, h = cv2.boundingRect(contour)
        return w > 2 * h





    def run(self):
        rospy.spin()



if __name__ == '__main__':
    try:
        color_lower = [0, 100, 100]
        color_upper = [10, 255, 255]

        color = input("Red: 1, Orange: 2, Yellow: 3, Blue: 4, Purple: 5\n")
        if color == 1: # Red
            # color_lower = [150, 50, 50]
            # color_upper = [360, 255, 255]
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
