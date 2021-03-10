#!/usr/bin/env python
# driver_node
# Zirui Zang
# 20200315
from __future__ import print_function
import sys
import math
import numpy as np
import rospy
import zmq
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import csv

def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    A = np.frombuffer(msg, dtype=md['dtype'])
    return A.reshape(md['shape'])

def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)

class driver_node:
    def __init__(self):
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_1'
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

def main(args):
    rospy.init_node("driver_node", anonymous=True)
    node = driver_node()
    angle_factor = 0.1

    context_box = zmq.Context()
    socket_box = context_box.socket(zmq.SUB)
    socket_box.setsockopt(zmq.SUBSCRIBE, b"")
    socket_box.setsockopt(zmq.RCVHWM, 1)
    socket_box.connect("tcp://192.168.1.5:5557")
    print("Collecting bboxs...")

    waypoints = np.loadtxt('/home/nvidia/catkin_ws/src/f110_avp/src/way.csv', delimiter=",")
    print(waypoints.shape)
    section2 = 0
    while not rospy.is_shutdown():
        if socket_box.poll(timeout = 1) != 0:
            bbox = recv_array(socket_box)
            pose = np.zeros((1, 3))
            pose[0, 0:2] = bbox[8, 0:2]
            current_theta = -(bbox[9, 0] + np.pi/2)
            distances = np.sqrt(np.power(waypoints[:, 0] - pose[0, 0], 2) + np.power(waypoints[:, 1] - pose[0, 1], 2))
            point_now_ind = np.argmin(distances)
            angle_factor = distances[point_now_ind]/2
            angle_factor = np.min([0.2, angle_factor])

#            print(pose)
            if point_now_ind < waypoints.shape[0]-3 and section2 == 0:
                point_now_ind += 3
                waypoint_x = waypoints[point_now_ind, 0]
                waypoint_y = waypoints[point_now_ind, 1]
                rot_waypoint_x = (waypoint_x - pose[0, 0]) * np.cos(-current_theta) - (waypoint_y - pose[0, 1]) * np.sin(-current_theta)
                rot_waypoint_y = (waypoint_x - pose[0, 0]) * np.sin(-current_theta) + (waypoint_y - pose[0, 1]) * np.cos(-current_theta)
                steering_angle = angle_factor * (2 * rot_waypoint_y) / (rot_waypoint_x ** 2 + rot_waypoint_y ** 2)


                if distances[point_now_ind] < 0.15:
                    steering_smooth = distances[point_now_ind] * 0.8
                    steering_angle = np.min([steering_angle, steering_smooth])
                    steering_angle = np.max([steering_angle, -steering_smooth])
                steering_angle = np.min([steering_angle, 0.4189])
                steering_angle = np.max([steering_angle, -0.4189])

                print(point_now_ind, distances[point_now_ind], angle_factor, steering_angle)

                nominal_speed = 0.9
                angle_speed = 0.5
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = steering_angle
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = 'ouster_os1'
                drive_msg.drive.speed = nominal_speed - (nominal_speed - angle_speed) * np.abs(steering_angle)/0.4189
                node.drive_pub.publish(drive_msg)

            else:
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = 0
                drive_msg.header.stamp = rospy.Time.now()
                drive_msg.header.frame_id = 'ouster_os1'
                drive_msg.drive.speed = 0
                node.drive_pub.publish(drive_msg)





if __name__ == '__main__':
    main(sys.argv)
