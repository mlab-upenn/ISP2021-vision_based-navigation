# zmq_inference.py
# Zirui Zang
# 20200224

import zmq
import io as sysio
import os
import sys
import time
from pathlib import Path
import numpy as np
import second.core.box_np_ops as box_np_ops
from second.pytorch.inference import TorchInferenceContext

def Preprocess(points_raw_load, scale_up):
    points = points_raw_load
    points = np.transpose(points)
    # print(points.shape)
    
    points = points[np.where( (points[:, 1] > -1.5) & (points[:, 1] < 1.5) )]
    shift_x = np.min(points[:, 0])
    shift_z = np.min(points[:, 2])
    points[:, 0] -= shift_x
    # points[:, 2] -= shift_z
    points[:, 2] += -0.12
    # points[:, 2] += -0.14
    points = points[np.where( points[:, 2] > 0 )]
    points = points[np.where( points[:, 3] > 100 )]
    points[:, 3] = 0
    # print(shift_x, shift_z)

    points_range = [np.max(points[:, 0]), np.max(points[:, 1]), np.max(points[:, 2]), np.min(points[:, 0]), np.min(points[:, 1]), np.min(points[:, 2])]
    # print(points.shape)
    # print('points_range', points_range)
    points = np.array(points) * scale_up

    return points, points_range, shift_x, shift_z

def BuildVoxelNet():
    config_path = Path('/home/lucerna/MEGAsync/project/AVP/second/configs/xyres_16.proto')
    ckpt_path = Path('/home/lucerna/MEGAsync/project/AVP/second/voxelnet-331653.tckpt')
    inference_ctx = TorchInferenceContext()
    inference_ctx.build(config_path)
    inference_ctx.restore(ckpt_path)
    return inference_ctx

def PointPillarsInference(inference_ctx, points, points_range, scale_up, shift_x, shift_z):
    if points.shape[0] < 100:
        return None
    inputs = inference_ctx.get_inference_input_dict(points)
    with inference_ctx.ctx():
        predictions_dicts = inference_ctx.inference(inputs)
    # print(predictions_dicts)
    detection_anno = predictions_dicts[0]
    if detection_anno["box3d_lidar"] is None:
        return None
    dt_box_lidar = np.array([detection_anno["box3d_lidar"].detach().cpu().numpy()])[0]
    scores = np.array([detection_anno["scores"].detach().cpu().numpy()])[0]
    # print(scores)

    # filter by score
    keep_list = np.where(scores > 0.4)[0]
    dt_box_lidar = dt_box_lidar[keep_list, :]
    scores = scores[keep_list]
    dt_box_lidar[:, :6] /= scale_up
    dt_box_lidar[:, 1] += shift_x
    dt_box_lidar[:, 2] -= -0.12
    
    dt_box_lidar_big = dt_box_lidar.copy()
    # dt_box_lidar_big[:, 4] *= 1.1 # length 
    dt_box_lidar_big[:, 3] *= 1.1 # width
    # dt_box_lidar[:, 4] *= 1.1 # length 
    dt_box_lidar[:, 3] *= 1.2 # width

    dt_boxes_corners = box_np_ops.center_to_corner_box3d(
        dt_box_lidar[:, :3],
        dt_box_lidar[:, 3:6],
        dt_box_lidar[:, 6],
        origin=[0.5, 0.5, 0],
        axis=2)

    dt_boxes_corners_big = box_np_ops.center_to_corner_box3d(
        dt_box_lidar_big[:, :3],
        dt_box_lidar_big[:, 3:6],
        dt_box_lidar_big[:, 6],
        origin=[0.5, 0.5, 0],
        axis=2)

    # filter bbox by its center
    centers = dt_box_lidar[:, :3]
    keep_list = np.where((centers[:, 0] < points_range[0]) & (centers[:, 0] > points_range[3]) & \
                            (centers[:, 1] < points_range[1]) & (centers[:, 1] > points_range[4]))[0]
    dt_boxes_corners = dt_boxes_corners[keep_list, :, :]
    dt_boxes_corners_big = dt_boxes_corners_big[keep_list, :]
    # dt_box_lidar = dt_box_lidar[keep_list]
    scores = scores[keep_list]

    num_dt = dt_boxes_corners.shape[0]

    if num_dt == 0:
        print('miss')
        return None
    boxes_select = np.zeros((18, 3))
    boxes_select[0:8, :] = dt_boxes_corners[0, :, :] # tight bbox
    boxes_select[8:16, :] = dt_boxes_corners_big[0, :, :]   # big bbox
    boxes_select[16, :] = dt_box_lidar[keep_list[0], :3]    # center position
    boxes_select[17, 0] = dt_box_lidar[keep_list[0], 6]    # orientation
    print('scores: ', scores, 'angles: ', dt_box_lidar[keep_list, 6])
    # print(dt_box_lidar[keep_list[0], 6])

    return boxes_select


def send_array(socket, A, flags=0, copy=True, track=False):
    """send a numpy array with metadata"""
    md = dict(
        dtype = str(A.dtype),
        shape = A.shape,
    )
    socket.send_json(md, flags|zmq.SNDMORE)
    return socket.send(A, flags, copy=copy, track=track)

def recv_array(socket, flags=0, copy=True, track=False):
    """ZMQ recv a numpy array"""
    md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    buf = bytes(memoryview(msg))
    A = np.frombuffer(buf, dtype=md['dtype'])
    return A.reshape(md['shape'])

def main():
    inference_ctx = BuildVoxelNet()

    context = zmq.Context()
    pc_socket = context.socket(zmq.SUB)
    pc_socket.setsockopt(zmq.SUBSCRIBE, b"")
    pc_socket.setsockopt(zmq.RCVHWM, 1)
    print("Collecting point clouds...")
    pc_socket.connect("tcp://localhost:5556")
    pc_socket.setsockopt(zmq.CONFLATE, 1)

    context_result = zmq.Context()
    socket_result = context_result.socket(zmq.PUB)
    socket_result.setsockopt(zmq.SNDHWM, 1)
    socket_result.bind("tcp://*:5557")
    scale_up = 7.0
    while True:
        points_raw = recv_array(pc_socket)
        # print('points_raw', points_raw.shape[1])
        if points_raw.shape[1] < 2000:
            continue
        points, points_range, shift_x, shift_z = Preprocess(points_raw, scale_up)
        boxes_select = PointPillarsInference(inference_ctx, points, points_range, scale_up, shift_x, shift_z)
        # print(boxes_select)
        if boxes_select is not None:
            send_array(socket_result, boxes_select)

if __name__ == '__main__':
    main()
