import copy
from typing import List, Optional, Tuple
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from scipy.spatial.transform import Rotation as R


def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    Nq = w*w + x*x + y*y + z*z
    if Nq < np.finfo(float).eps:
        return np.identity(3)
    s = 2.0/Nq
    X = x*s; Y = y*s; Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    return np.array(
        [[1.0-(yY+zZ), xY-wZ, xZ+wY],
         [xY+wZ, 1.0-(xX+zZ), yZ-wX],
         [xZ-wY, yZ+wX, 1.0-(xX+yY)]])

def quaternion2rot(quaternion):
    r = R.from_quat(quaternion)
    rot = r.as_matrix()
    return rot


def read_pcd(pcd_file):
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)
    return points


def project(points, image, M1, M2):
    """
    points: Nx3
    image: opencv img, 表示要投影的图像
    M1: 内参矩阵 K, 4*4
    M2: 外参矩阵， 4*4

    return: points 在像素坐标系下的坐标 N*4, 实际只用 N*2

    """
    resolution = image.shape
    # print(resolution)

    coords = points[:, 0:3]
    ones = np.ones(len(coords)).reshape(-1, 1)
    # 转换为齐次坐标
    coords = np.concatenate([coords, ones], axis=1)
    print(coords.shape)

    # transform = copy.deepcopy(M1 @ M2).reshape(4, 4)
    # coords = coords @ transform.T
    # coords = coords[np.where((coords[:, 2] > 0.1) & (coords[:, 2] < 10))]

    # 计算点云在相机坐标系下的坐标
    coords_current = coords @ M2.T 
    coords_current = coords_current[np.where((coords_current[:, 2] > 0) & (coords_current[:, 2] < 6))]
    # coords_current = coords_current[np.where((coords_current[:, 2] > 0))]
    print(coords_current.shape)

    # 计算点云在像素坐标系下的坐标
    coords_pixel = coords_current @ M1.T
    coords_pixel = coords_pixel[np.where(coords_pixel[:, 2] > 0)]
    print(coords_pixel.shape)
    # 深度归一化
    coords_pixel[:, 2] = np.clip(coords_pixel[:, 2], a_min=1e-5, a_max=1e5)
    coords_pixel[:, 0] /= coords_pixel[:, 2]
    coords_pixel[:, 1] /= coords_pixel[:, 2]

    # 筛除超出图像范围的点
    coords = coords_pixel[np.where(coords_pixel[:, 0] > 0)]
    coords = coords_pixel[np.where(coords_pixel[:, 0] < resolution[1])]
    coords = coords_pixel[np.where(coords_pixel[:, 1] > 0)]
    coords = coords_pixel[np.where(coords_pixel[:, 1] < resolution[0])]

    return coords


def show_with_opencv(image, coords=None):
    """
    image: opencv image
    coords: 像素坐标系下的点, N*4
    """
    canvas = image.copy()
    # cv2.putText(canvas,
    #             text='project_lidar2img',
    #             org=(90, 180),
    #             fontFace=cv2.FONT_HERSHEY_PLAIN,
    #             fontScale=12.0,
    #             thickness=10,
    #             color=(0, 0, 255))
    canvas = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)
    # 画点
    # 根据canvas的第3维进行颜色映射，得到canvas[2][index]对应的rgb三通道颜色

    if coords is not None:
        for index in range(coords.shape[0]):
            p = (int(coords[index, 0]), int(coords[index, 1]))
            cv2.circle(canvas, p, 2, color=[0, 0, 255], thickness=1)
    canvas = canvas.astype(np.uint8)
    canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

    print(canvas.shape)
    canvas = cv2.resize(canvas, (1280, 720))
    cv2.namedWindow("image")  # 创建一个image的窗口
    cv2.imshow("image", canvas)  # 显示图像
    cv2.waitKey(0)  # 默认为0，无限等待


if __name__ == '__main__':
    points = read_pcd('/home/zmxj/code/Datasets/20240422cam_infra_lidar/output/2024-04-22-16-59-20_tree/scans.pcd')
    img = cv2.imread('/home/zmxj/code/Datasets/20240422cam_infra_lidar/output/2024-04-22-16-59-20_tree/infra_images/1713776367614057716_1.png')
    # undistort
    
    M1 = np.array([[648.2316170199845, 0.0000000e+00, 650.0226486382421, 0.0000000e+00],
                   [0.0000000e+00, 650.0226486382421, 362.2907228616518, 0.0000000e+00],
                   [0.0000000e+00, 0.0000000e+00, 1.0000000e+00, 0.0000000e+00],
                   [0.0000000e+00, 0.0000000e+00, 0.0000000e+00, 1.0000000e+00]],
                  dtype=np.float32)
    T_imu2cam = np.array([[0.13155732, -0.99056855, 0.03829638, -0.24086951],
                          [-0.00218127, -0.03892131, -0.9992399, 0.95844566],
                          [0.99130617, 0.13137379, -0.00728109, -1.57839183],
                          [0.000000, 0.000000, 0.000000, 1.000000]],
                         dtype=np.float32)
    T_lidar2imu = np.array([[1.0, 0.000000, 0.000000, -0.011],
                            [0.000000, 1.0, 0.000000, -0.2329],
                            [0.000000, 0.000000, 1.0, 0.4412],
                            [0.000000, 0.000000, 0.000000, 1.000000]],
                           dtype=np.float32)
    T_lidar2cam = T_imu2cam @ T_lidar2imu

    # 1713776367.682592630 3.243489795 0.981876918 0.290253852 -0.041104698 -0.038485699 -0.117327907 0.991495546
    t = [3.243489795, 0.981876918, 0.290253852]
    q_xyzw = [-0.041104698, -0.038485699, -0.117327907, 0.991495546]
    # 将q_xyzw归一化
    q_xyzw = q_xyzw / np.linalg.norm(q_xyzw)
    q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
    # R_orign2current = quaternion_to_rotation_matrix(q_wxyz)
    R_orign2current = quaternion2rot(q_xyzw)
    # print(R_orign2current * np.linalg.inv(R_orign2current))
    # R_orign2current = R_orign2current.T
    # t = np.dot(-R_orign2current, t)

    T_orign2current = np.array([[R_orign2current[0, 0], R_orign2current[0, 1], R_orign2current[0, 2], t[0]],
                                [R_orign2current[1, 0], R_orign2current[1, 1], R_orign2current[1, 2], t[1]],
                                [R_orign2current[2, 0], R_orign2current[2, 1], R_orign2current[2, 2], t[2]],
                                [0.000000, 0.000000, 0.000000, 1.000000]],
                               dtype=np.float32)
    # 输出T_orign2current的rank
    # print(np.linalg.matrix_rank(T_orign2current))
    T_orign2current = np.linalg.inv(T_orign2current)
    # print(T_orign2current @ np.linalg.inv(T_orign2current))
    # print(T_orign2current)
    T_orign2camera = T_lidar2cam @ T_orign2current
    coords = project(points, img, M1, T_orign2camera)
    show_with_opencv(img, coords=coords)

