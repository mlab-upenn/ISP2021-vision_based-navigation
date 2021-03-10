# occupancy_grid.py
# Zirui Zang
# 20200229

import zmq
import numpy as np
from matplotlib import pyplot as plt
from skimage.transform import rescale
from skimage.draw import polygon, polygon_perimeter
from skimage.measure import label
import second.core.box_np_ops as box_np_ops


class OccupancyGrid():
    def __init__(self, dim1, dim2):
        self.matrix = np.zeros((dim1, dim2))
        self.image = np.ones((dim1, dim2, 3), dtype=np.uint8) * 225

    def update_image(self):
        matrix = self.matrix
        self.image[np.where(matrix == 0)] = [255, 255, 255]
        self.image[np.where(matrix == 1)] = [204, 0, 0]
        self.image[np.where(matrix == 2)] = [0, 125, 255]
        self.image[np.where(matrix == 3)] = [0, 51, 153]

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

def find_coords(input, resolution, is_y = 0):
    if is_y == 0:
        return (np.floor(input * resolution))
    else:
        return (np.floor(input * resolution) + is_y)

def find_neighbours(coord_x, coord_y, limit_x, limit_y, option = 0):
    if option == 0:
        neighbors = np.array([[np.min([limit_x-1, coord_x+1]), coord_y], \
                            [np.max([0, coord_x-1]), coord_y], \
                            [coord_x, np.min([limit_y-1, coord_y+1])], \
                            [coord_x, np.max([0, coord_y-1])], \
                            [np.min([limit_x-1, coord_x+1]), np.min([limit_y-1, coord_y+1])], \
                            [np.min([limit_x-1, coord_x+1]), np.max([0, coord_y-1])], \
                            [np.max([0, coord_x-1]), np.min([limit_y-1, coord_y+1])], \
                            [np.max([0, coord_x-1]), np.max([0, coord_y-1])]])
    elif option == 1:
        neighbors = np.array([[np.min([limit_x-1, coord_x+1]), np.min([limit_y-1, coord_y+1])], \
                            [np.min([limit_x-1, coord_x+1]), np.max([0, coord_y-1])], \
                            [np.max([0, coord_x-1]), np.min([limit_y-1, coord_y+1])], \
                            [np.max([0, coord_x-1]), np.max([0, coord_y-1])]])
    return neighbors

def main():
    # np.set_printoptions(threshold=np.inf)
    # pc_file = '/home/lucerna/MEGA/project/AVP/2427439726050.npy'
    # pc = np.transpose(np.load(pc_file))

    context_cloud = zmq.Context()
    socket_cloud = context_cloud.socket(zmq.SUB)
    socket_cloud.setsockopt(zmq.SUBSCRIBE, b"")
    print("Collecting point clouds...")
    socket_cloud.connect("tcp://localhost:5556")

    context_box = zmq.Context()
    socket_box = context_box.socket(zmq.SUB)
    socket_box.setsockopt(zmq.SUBSCRIBE, b"")
    print("Collecting bboxs...")
    socket_box.connect("tcp://localhost:5557")

    context_grid = zmq.Context()
    socket_grid = context_grid.socket(zmq.PUB)
    socket_grid.bind("tcp://*:5558")

    x_clip = np.array([0, 1.5])
    y_clip = np.array([-1.5, 1.5])
    z_clip = 0.1
    grid_res = 100
    object_res = 50

    # create occupancy grid
    dim_x = int((x_clip[1] - x_clip[0]) * grid_res)
    dim_y = int((y_clip[1] - y_clip[0]) * grid_res)
    dim_x_object = int((x_clip[1] - x_clip[0]) * object_res)
    dim_y_object = int((y_clip[1] - y_clip[0]) * object_res)
    object_matrix = np.zeros([dim_x_object, dim_y_object])
    car_matrix = np.zeros([dim_x_object, dim_y_object])
    car_matrix_object = np.zeros([dim_x_object, dim_y_object])
    pc_in = None
    bbox = None
    
    while True:
        if socket_cloud.poll(timeout = 10) != 0:
            pc_in = np.transpose(recv_array(socket_cloud))
        if socket_box.poll(timeout = 10) != 0:
            bbox = recv_array(socket_box)
        occupancy_grid = OccupancyGrid( dim_x, dim_y )

        if bbox is not None:
            # add detection in the grid
            rect_x = np.zeros((4, ))
            rect_y = np.zeros((4, ))
            rect_x[0] = find_coords(bbox[0+8, 0], object_res)
            rect_y[0] = find_coords(bbox[0+8, 1], object_res, dim_y_object/2)
            rect_x[1] = find_coords(bbox[4+8, 0], object_res)
            rect_y[1] = find_coords(bbox[4+8, 1], object_res, dim_y_object/2)
            rect_x[2] = find_coords(bbox[6+8, 0], object_res)
            rect_y[2] = find_coords(bbox[6+8, 1], object_res, dim_y_object/2)
            rect_x[3] = find_coords(bbox[2+8, 0], object_res)
            rect_y[3] = find_coords(bbox[2+8, 1], object_res, dim_y_object/2)
            car_coords_x, car_coords_y = np.array(polygon(rect_x, rect_y, shape = (dim_x_object, dim_y_object)))
            car_matrix = np.zeros([dim_x_object, dim_y_object])
            car_matrix_object = np.zeros([dim_x_object, dim_y_object])
            car_matrix[car_coords_x, car_coords_y] = 1

            rect_x[0] = find_coords(bbox[0, 0], grid_res)
            rect_y[0] = find_coords(bbox[0, 1], grid_res, dim_y/2)
            rect_x[1] = find_coords(bbox[4, 0], grid_res)
            rect_y[1] = find_coords(bbox[4, 1], grid_res, dim_y/2)
            rect_x[2] = find_coords(bbox[6, 0], grid_res)
            rect_y[2] = find_coords(bbox[6, 1], grid_res, dim_y/2)
            rect_x[3] = find_coords(bbox[2, 0], grid_res)
            rect_y[3] = find_coords(bbox[2, 1], grid_res, dim_y/2)
            car_peri_coords_x, car_peri_coords_y = np.array(polygon_perimeter(rect_x, rect_y, shape = (dim_x, dim_y), clip = True))
        
        if pc_in is not None:
            # add objects in the grid
            pc = pc_in[np.where( (pc_in[:, 1] > y_clip[0]) & (pc_in[:, 1] < y_clip[1]) )]
            pc[:, 0] -= np.min(pc[:, 0])
            pc = pc[np.where( pc[:, 0] < x_clip[1] )]
            pc[:, 2] += -z_clip
            pc = pc[np.where( (pc[:, 2] > 0) )]
            pc = pc[np.where( (pc[:, 3] > 100) )]

            pc_grid = pc[:, 0:2]
            pc_grid[:, 0] = (np.floor(pc_grid[:, 0] * object_res))
            pc_grid[:, 1] = (np.floor(pc_grid[:, 1] * object_res) + dim_y_object/2)
            pc_grid = pc_grid.astype(int)
            pc_grid, counts = np.unique(pc_grid, return_counts = True, axis = 0)
            pc_grid = pc_grid[np.where(counts > grid_res/object_res)]

            object_matrix = np.zeros([dim_x_object, dim_y_object])
            object_matrix[pc_grid[:, 0], pc_grid[:, 1]] = 1
            
            # inflate the object matrix 
            # for ind in range(pc_grid.shape[0]):
            #     neighbors = find_neighbours(pc_grid[ind, 0], pc_grid[ind, 1], dim_x_object, dim_y_object, option = 0)
            #     for neighbor in neighbors:
            #         object_matrix[neighbor[0], neighbor[1]] = 1
            # pc_grid = np.transpose(np.array(np.where(object_matrix == 1)))
            
            for ind in range(pc_grid.shape[0]):
                if car_matrix[pc_grid[ind, 0], pc_grid[ind, 1]] == 1:
                    car_matrix_object[pc_grid[ind, 0], pc_grid[ind, 1]] = 1
                else:
                    neighbors = find_neighbours(pc_grid[ind, 0], pc_grid[ind, 1], dim_x_object, dim_y_object, option = 0)
                    for neighbor in neighbors:
                        if car_matrix[neighbor[0], neighbor[1]] == 1:
                            car_matrix_object[pc_grid[ind, 0], pc_grid[ind, 1]] = 1
                            continue

            object_matrix = rescale(object_matrix, grid_res/object_res, anti_aliasing=False)
            object_matrix[np.where(object_matrix > 0)] = 1
            car_matrix_object = rescale(car_matrix_object, grid_res/object_res, anti_aliasing=False)
            car_matrix_object[np.where(car_matrix_object > 0)] = 1

            matrix_labels, num = label(object_matrix, connectivity=2, return_num=True)
            find_flag = 0
            for num_ind in range(num+1):
                label_xy = np.array(np.where(matrix_labels == num_ind))
                if num_ind != 0:
                    for ind in range(label_xy.shape[1]):
                        if car_matrix_object[label_xy[0, ind], label_xy[1, ind]] == 1:
                            car_matrix_object[label_xy[0, :], label_xy[1, :]] = 1
                            find_flag = 1
                        if find_flag == 1:
                            break
                        # print(label_xy.shape[1], ind)

            

        if pc_in is not None and bbox is not None:
            occupancy_grid.matrix[np.where(object_matrix > 0)] = 1
            occupancy_grid.matrix[np.where(car_matrix_object > 0)] = 2
            occupancy_grid.matrix[car_peri_coords_x, car_peri_coords_y] = 3 # perimeter line
            occupancy_grid.update_image()
            send_array(socket_grid, occupancy_grid.image)

            # fig2 = plt.figure()
            # plt.imshow(matrix_labels)
            # fig1 = plt.figure()
            # plt.imshow(car_matrix_object)
            # fig3 = plt.figure()
            # plt.imshow(map1)
            # plt.show()


if __name__ == '__main__':
    main()