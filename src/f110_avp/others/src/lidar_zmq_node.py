#!/usr/bin/env python
# lidar_zmq_node.py
# Zirui Zang
# 20200224

from __future__ import print_function
import rospy
import ros_numpy
import geometry_msgs.msg
import zmq
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import time
# import numba
from numba import jit

class lidar_zmq_node:
    def __init__(self):
        lidarscan_topic = '/os1_cloud_node/points'
        # lidarscan_topic = 'pc_transformed'
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, PointCloud2, self.lidar_callback)
        # self.count = 0
        # self.points_save = np.zeros((64*1024, 3))

        self.pc_pub = rospy.Publisher('pc_transformed', PointCloud2, queue_size = 1)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.bind("tcp://*:5556")
        # self.ind_list = np.array(range(1, 2048, 4))
        # self.ind_list2 = np.array(range(0, 64, 3))
        self.m = self.rotation_matrix([0, 1, 0], -25/180.0*np.pi)
        # self.m = self.rotation_matrix([0, 1, 0], -22/180.0*np.pi)

    def send_array(self, socket, A, flags=0, copy=True, track=False):
        """send a numpy array with metadata"""
        md = dict(
            dtype = str(A.dtype),
            shape = A.shape,
        )
        socket.send_json(md, flags|zmq.SNDMORE)
        return socket.send(A, flags, copy=copy, track=track)

    def rotation_matrix(self, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        axis = np.asarray(axis)
        axis = axis / np.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    def rotation(self, input):
        return np.dot(self.m, input)

    def lidar_callback(self, msg):
        # time1 = time.time()
        pc = ros_numpy.numpify(msg)
        # print(pc.shape)
        pc = pc.flatten()
        # print(pc.shape)
        # pc = pc[:, self.ind_list].flatten()
        # pc = pc[self.ind_list2, :].flatten()
        
        pc = pc[np.where( (pc['x'] < -1.5) & (pc['y'] > -1.5) & (pc['y'] < 1.5) )]
        pc_cropped = np.empty((3, pc['x'].shape[0]))
        pc_cropped[0, :] = pc['x']
        pc_cropped[1, :] = pc['y']
        pc_cropped[2, :] = pc['z']
        pc_transformed = self.rotation(pc_cropped)
        pc_transformed = np.concatenate((pc_transformed, np.expand_dims(pc['intensity'], axis=0)), axis=0)
        # print(pc_transformed.shape)
        pc_transformed[0, :] += 3
        pc_transformed[2, :] += 1.47
        pc['x'] = pc_transformed[0, :]
        pc['y'] = pc_transformed[1, :]
        pc['z'] = pc_transformed[2, :]

        msg_transformed = ros_numpy.msgify(PointCloud2, pc) 
        msg_transformed.header.frame_id = '/os1_lidar'
        self.pc_pub.publish(msg_transformed)
        self.send_array(self.socket, pc_transformed)
        
def main():
    rospy.init_node("lidar_zmq_node", anonymous=True)
    node = lidar_zmq_node()
    print('Sending point cloud...')
    rospy.spin()

if __name__ == '__main__':
    main()