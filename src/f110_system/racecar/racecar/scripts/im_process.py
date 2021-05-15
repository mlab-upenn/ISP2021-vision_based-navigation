#!/usr/bin/env python
from __future__ import print_function
import sys
import math
#ROS Imports
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import numpy as np

import cv2
class Image_drive:
    def __init__(self):
        # Topics & Subscriptions,Publishers
        image_topic = '/cv_camera/image_rect_color'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'

        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)  # TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)  # TODO

    def showImage(self, img):
        cv2.imshow('image', img)
        cv2.waitKey(1)

    def image_callback(self,msg):
       # convert sensor_msgs/Image to OpenCV Image
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        drawImg = orig
        # resize image (half-size) for easier processing
        resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
        drawImg = resized
        # convert to single-channel image
        # gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        # drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)


        # drawImg = cv2.cvtColor(gray, cv2.COLOR_BGR2HSV)
        # threshold grayscale to binary (black & white) image
        # threshVal = 75
        # ret,thresh = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY)
        lower_black = np.array([110, 50, 0])
        upper_black = np.array([130, 255, 255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_black, upper_black)
        hist = np.sum(mask, axis=0)
        max_black_idx = np.argmax(hist)
        width = hsv.shape[1]
        height = hsv.shape[0]
        abs_value = abs(max_black_idx - width/2)
        idx_sign = -np.sign(max_black_idx - width/2)

        if abs_value < 45:
            velocity = 0.5
            angle_forward = 0
            print("drive straight")
        elif abs_value >= 45 and abs_value <= 100:
            velocity = 0.5
            angle_forward = idx_sign*math.radians(8)
            if idx_sign == -1:
                print("on the left, drive right")
            elif idx_sign == 1:
                print("on the right, drive left")
        elif abs_value >= 45 and abs_value < 110:
            velocity = 0.5
            angle_forward = idx_sign*math.radians(8+abs_value*0.3)
            if idx_sign == -1:
                print("on the left, drive right")
            elif idx_sign == 1:
                print("on the right, drive left")
        elif abs_value >= 110 and abs_value < 150:
            velocity = 0.5
            angle_forward = idx_sign*math.radians(25)
            if idx_sign == -1:
                print("on the left, drive right")
            elif idx_sign == 1:
                print("on the right, drive left")
        else:
            velocity = 0


        # Publish Drive message
        rospy.loginfo("------------------")
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle_forward
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        self.showImage(mask)


def main(args):
    rospy.init_node("LaneDrive_node", anonymous=True)
    rospy.loginfo('lane_line_driving node started')
    imgdrive = Image_drive()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)