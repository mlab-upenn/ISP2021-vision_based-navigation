#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 1.0 #TODO
kd = 0.09   #TODO
ki = 0.0  #TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
# angle = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.9 #0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
THETA0 = 90
THETA1 = 45 #angle in degree
L = 1.0


class LaneDte:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        # drive_topic = '/nav'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        #TODO: Subscribe to LIDAR
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size = 1)
        #TODO: Publish to drive
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 1)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right <- wrong
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        #THETA0=90,THETA1=45;
        flag0 = 0
        flag1 = 0
        # theta0 = math.radians(THETA0)
        # theta1= math.radians(THETA1)
        # print("Angle min: ", math.degrees(data.angle_min))
        # print("Angle max: ",  math.degrees(data.angle_max))
        for i in range(0,len(data.ranges)):
            theta = data.angle_min + data.angle_increment * i
            if theta < math.radians(-135) or theta > math.radians(135):
                continue

            if theta >= math.radians(THETA1) and flag1 == 0 and data.ranges[i] >= data.range_min and data.ranges[i] <= data.range_max and np.isinf(data.ranges[i])==False and np.isnan(data.ranges[i])==False:
                range1 = data.ranges[i]
                angle1 = theta
                flag1 = 1
            if theta >= math.radians(THETA0) and flag0 == 0 and data.ranges[i] >= data.range_min and data.ranges[i] <= data.range_max and np.isinf(data.ranges[i])==False and np.isnan(data.ranges[i])==False:
                range0 = data.ranges[i]
                angle0 = theta
                flag0 = 1
                break
            i+=1
        a = range1
        b = range0
        return a,b, angle0, angle1

    def lidar_callback(self, data):
        """
        """
        # print("range num", len(data.ranges))
        # print("range interval", math.degrees(data.angle_increment))
        print("right distance: ",data.ranges[180])
        print("left distance: ",data.ranges[900])
        print("front distance: ",data.ranges[540])
        a,b,angle0,angle1 = self.getRange(data,THETA1)
        theta = angle0 - angle1
        # rospy.loginfo("angle1: %f", angle1)
        # rospy.loginfo("theta: %.2f(d)", math.degrees(theta))
        # rospy.loginfo("range_90: %f", b)
        # rospy.loginfo("range_45: %f", a)
        # rospy.loginfo("theta: %f", theta)
        alpha = math.atan((a * math.cos(theta)-b)/(a*math.sin(theta)))
        # rospy.loginfo("alpha: %.2f(d)", math.degrees(alpha))
        Dt1 = b * math.cos(alpha)
        # rospy.loginfo("Dt1: %f", Dt1)
        Dt2 = Dt1 + L * math.sin(alpha)
        # rospy.loginfo("Dt2: %f", Dt2)
        error = DESIRED_DISTANCE_LEFT - Dt2
        #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        # global angle
        angle = 0.0
        # rospy.loginfo("error: %.2f(m)", error)
        dedt = error - prev_error
        # rospy.loginfo("dedt: %.2f(m)", dedt)
        Vangle = kp * error + kd * dedt
        angle = -Vangle
        if angle > math.radians(100) :
            angle = math.radians(100)
        elif angle < math.radians(-100):
            angle = math.radians(-100)
        prev_error = error
        # if angle > 0:
        #     rospy.loginfo("steering angle: +")
        # elif angle < 0:
        #     rospy.loginfo("steering angle: -")
        rospy.loginfo("st_angle: %.2f(d)", math.degrees(angle))

        #TODO: Use kp, ki & kd to implement a PID controller for
        if abs(angle) < math.radians(10):
            rospy.loginfo("drive staight")
            # velocity = 1.5
            velocity = 0.8
        elif abs(angle) >= math.radians(10) and abs(angle) < math.radians(20):
            # velocity = 1.0
            velocity = 0.5
        else:
            rospy.loginfo("drive corner")
            # velocity = 0.5
            velocity = 0.3

        rospy.loginfo("------------------")
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        return 0.0 

# def main(args):
#     rospy.init_node("WallFollow_node", anonymous=True)
#     wf = WallFollow()
#     rospy.sleep(0.1)
#     rospy.spin()

# if __name__=='__main__':
# 	main(sys.argv)

if __name__ == '__main__':
    rospy.init_node("WallFollow_node", anonymous=True)
    WallFollow()
    # rospy.sleep(0.1)
    rospy.spin()
