#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        # drive_topic = '/nav'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window --- ?
            2.Rejecting high values (eg. > 3m)
        """
        max_range = 3.0
        inter = np.zeros_like(ranges)
        for i in range(0,len(ranges)):
            if np.isinf(ranges[i]) or ranges[i]>max_range:
                inter[i] = max_range
            elif np.isnan(ranges[i]):
                inter[i] = 0.0
            else:
                inter[i] = ranges[i]
        #truncate to front 180 filed of view
        proc_ranges = np.array(inter[180:900])
        print("LENGHT",len(proc_ranges))
        # print("LIDAR TEST",  proc_ranges[0])
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_gap = 0     #length of current max gap
        curr_gap = 0    #length of current gap

        index = 0

        max_gap_end_ind = 0
        curr_gap_end_ind = 0
        print("LEN free spce", len(free_space_ranges))
        while index<len(free_space_ranges):

            # max_gap_start_ind = index
            while index<len(free_space_ranges) and free_space_ranges[index] != 0:
                index += 1
                curr_gap += 1
                curr_gap_end_ind = index
            
            if curr_gap>max_gap:
                max_gap = curr_gap
                curr_gap = 0
                max_gap_end_ind = curr_gap_end_ind
                max_gap_start_ind = max_gap_end_ind - max_gap
            index += 1
        print("Max gap indeces : ", max_gap_start_ind, " & " ,max_gap_end_ind)
        return max_gap_start_ind, max_gap_end_ind
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        return None

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        angle_incr = data.angle_increment
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        min_pos = np.argmin(proc_ranges)
        min_dist = min(proc_ranges)
        print("MIN INDEX: ", min_pos)
        #Eliminate all points inside 'bubble' (set them to zero) 
        r_bubble = 0.5/2
        l_arc = min_dist * angle_incr
        l_arc_total = 0
        curr_index = min_pos
        while(l_arc_total<r_bubble and curr_index>0):
            proc_ranges[curr_index] = 0.0
            curr_index -= 1 #to the right
            l_arc_total += l_arc
        print("exit loop 111111")

        l_arc_total = 0
        curr_index = min_pos
        while(l_arc_total<r_bubble and curr_index<540):
            proc_ranges[curr_index] = 0.0
            curr_index += 1 #to the left
            l_arc_total += l_arc
        print("exit loop 22222")

        #Find max length gap 
        max_gap_start, max_gap_end = self.find_max_gap(proc_ranges)
        # print("MAX gap start", max_gap_start)
        # print("MAX gap end", max_gap_end)
        #Find the best point in the gap 
        #mid point
        best_point_ind = int((max_gap_start + max_gap_end)/2)
        print("Best point ind" , best_point_ind)
        #Publish Drive message
        velocity = 0.8
        direction = (best_point_ind + 180) * data.angle_increment
        print("Direction", direction - ((3*3.14)/4))
        steering_angle = (direction - ((3*3.14)/4))/2
        print('STEERING ANGLE', steering_angle)
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)