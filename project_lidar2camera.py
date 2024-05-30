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
    # coords_current = coords_current[np.where((coords_current[:, 2] > 0.20) & (coords_current[:, 2] < 15))]
    coords_current = coords_current[np.where((coords_current[:, 2] > 0.20 ) )]
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
    # 生成一个和image一样大小的画布，白色背景
    img_lidar = np.ones_like(image) * 255
    img_lidar_black = np.zeros_like(image) * 255
    # cv2.putText(canvas,
    #             text='project_lidar2img',
    #             org=(90, 180),
    #             fontFace=cv2.FONT_HERSHEY_PLAIN,
    #             fontScale=12.0,
    #             thickness=10,
    #             color=(0, 0, 255))
    canvas = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)
    # 画点
    # 创建一个颜色映射
    cmap = plt.get_cmap('jet')
    # 对coords[2]进行归一化
    coords_z_norm = colors.Normalize(vmin=coords[:, 2].min(), vmax=coords[:, 2].max())(coords[:, 2])
    if coords is not None:
        # 按照深度(z坐标)排序，使得远处的点先被绘制
        sorted_indices = np.argsort(coords[:, 2])[::-1]
        for index in sorted_indices:
            p = (int(coords[index, 0]), int(coords[index, 1]))
            # 应用颜色映射，得到RGB颜色
            rgb_color = cmap(coords_z_norm[index])[:3]
            # 将rgb_color的值从[0, 1]区间转换到[0, 255]区间，并转换为整数
            rgb_color = (np.array(rgb_color) * 255).astype(int)
            # 根据深度调整点的大小，使得近处的点大于远处的点
            radius = int(5 * (1 - coords_z_norm[index]))
            # radius = 2

            cv2.circle(canvas, p, radius, color=rgb_color.tolist(), thickness=-1)
            cv2.circle(img_lidar, p, radius, color=rgb_color.tolist(), thickness=-1)
            cv2.circle(img_lidar_black, p, radius, color=rgb_color.tolist(), thickness=-1)
    canvas = canvas.astype(np.uint8)
    canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

    print(canvas.shape)
    canvas = cv2.resize(canvas, (1280, 720))
    cv2.namedWindow("image")  # 创建一个image的窗口
    cv2.imshow("image", canvas)  # 显示图像
    cv2.waitKey(0)  # 默认为0，无限等待
    # 保存canvas和img_lidar
    cv2.imwrite('canvas.png', canvas)
    cv2.imwrite('img_lidar.png', img_lidar)
    cv2.imwrite('img_lidar_black.png', img_lidar_black)


if __name__ == '__main__':
    # datasets_tree
    # points = read_pcd('/home/zmxj/code/Datasets/20240422cam_infra_lidar/output/2024-04-22-16-59-20_tree/scans.pcd')
    # img = cv2.imread('/home/zmxj/code/Datasets/20240422cam_infra_lidar/output/2024-04-22-16-59-20_tree/infra_images/1713776380353760107_1.png')
    
    # datasets_box
    # points = read_pcd('/home/zmxj/code/Datasets/20240422cam_infra_lidar/output/2024-04-22-16-45-53_box/scans.pcd')
    # img = cv2.imread('/home/zmxj/code/Datasets/20240422cam_infra_lidar/output/2024-04-22-16-45-53_box/infra_images/1713775615829020319_1.png')
    
    # datasets_box_new
    # points = read_pcd('/home/zmxj/code/Datasets/20240529cam_lidar/output/20240529_d455_lidar/scans.pcd')
    # img = cv2.imread('/home/zmxj/code/Datasets/20240529cam_lidar/output/20240529_d455_lidar/infra_images/1716982960834713193_1.png')

    # datasets_box_new
    points = read_pcd('/home/zmxj/code/Datasets/20240529cam_lidar/output/20240529_tree/scans.pcd')
    img = cv2.imread('/home/zmxj/code/Datasets/20240529cam_lidar/output/20240529_tree/infra_images/1716992137778292235_1.png')

    # TODO: undistort
    # Distortion model: radtan
    # Distortion coefficients: [0.0008199309529270651, 0.0009753090923884344, 0.00019208353144597011, 0.0002669581142685641]
    
    M1 = np.array([[648.2316170199845, 0.0000000e+00, 650.0226486382421, 0.0000000e+00],
                   [0.0000000e+00, 638.6350168260949, 362.2907228616518, 0.0000000e+00],
                   [0.0000000e+00, 0.0000000e+00, 1.0000000e+00, 0.0000000e+00],
                   [0.0000000e+00, 0.0000000e+00, 0.0000000e+00, 1.0000000e+00]],
                  dtype=np.float32)
    T_imu2cam = np.array([[ 0.13155732, -0.99056855,  0.03829638, -0.24086951],
                          [-0.00218127, -0.03892131, -0.99923990,  0.95844566],
                          [ 0.99130617,  0.13137379, -0.00728109, -1.57839183],
                          [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]],
                         dtype=np.float32)
    T_lidar2imu = np.array([[1.0, 0.000000, 0.000000, -0.011],
                            [0.000000, 1.0, 0.000000, -0.2329],
                            [0.000000, 0.000000, 1.0, 0.4412],
                            [0.000000, 0.000000, 0.000000, 1.000000]],
                           dtype=np.float32)
    T_lidar2cam = T_imu2cam @ T_lidar2imu

    # timestamp t.x t.y t.z q.x q.y q.z q.w
    # tree 
    # 1713776380.315001249 0.449649423 -0.526561961 0.100077772 -0.027703782 0.055361529 -0.117364527 0.991157490
    # t = [0.449649423, -0.526561961, 0.100077772]
    # q_xyzw = [-0.027703782, 0.055361529, -0.117364527, 0.991157490]
    # box
    # 1713775558.892144918 3.390272918 -0.833027783 0.714059995 -0.010689520 -0.085178690 -0.029043469 0.995884934
    # 1713775615.824685097 13.627896289 0.313170908 2.998558731 0.054844431 -0.026343355 -0.800791394 0.595844996
    # 1713775604.557905436 9.208605812 -3.043218760 1.942622104 -0.038414712 -0.092868895 0.020914551 0.994717176
    # 1713775556.024746656 1.315466880 -0.273530509 0.228306446 0.035951976 -0.087664849 -0.034209703 0.994913075
    # 1713775554.458202124 0.107204243 -0.038100844 0.044389021 -0.000766078 -0.004780799 0.014499025 0.999883161
    # box_new
    # 1716982960.840327740 0.078888632 0.007249208 -0.005471448 -0.016008800 0.100572695 -0.061843653 0.992876736
    # 1716983052.909453392 14.844449372 3.998762430 -1.337474186 -0.000477630 0.063451639 0.371706026 0.926179406
    # 1716983046.441487312 14.816408095 4.030169645 -1.345039903 -0.003049507 0.060942450 0.476709328 0.876940668
    # 1716983042.975462675 13.407206870 3.907722110 -1.188876175 0.002422793 0.073501342 0.655459431 0.751641282
    # tree_new
    # 1716992137.762100935 24.597279009 -10.806595930 -0.618385338 -0.027517933 -0.023816320 -0.881140381 0.471452197
    # 1716992088.494944096 0.018604187 0.049228831 -0.207596469 0.022846804 -0.046619863 -0.058181004 0.996955156
    t = [24.597279009, -10.806595930, -0.618385338]   
    q_xyzw = [-0.027517933, -0.023816320, -0.881140381, 0.471452197] 
    # 将q_xyzw归一化
    q_xyzw = q_xyzw / np.linalg.norm(q_xyzw)
    # q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
    # R_orign2current = quaternion_to_rotation_matrix(q_wxyz)
    R_orign2current = quaternion2rot(q_xyzw)
    # print(R_orign2current * np.linalg.inv(R_orign2current))
    # R_orign2current = R_orign2current.T
    # t = np.dot(-R_orign2current, t)

    T_current2orign = np.array([[R_orign2current[0, 0], R_orign2current[0, 1], R_orign2current[0, 2], t[0]],
                                [R_orign2current[1, 0], R_orign2current[1, 1], R_orign2current[1, 2], t[1]],
                                [R_orign2current[2, 0], R_orign2current[2, 1], R_orign2current[2, 2], t[2]],
                                [0.000000, 0.000000, 0.000000, 1.000000]],
                               dtype=np.float32)
    T_orign2current = np.linalg.inv(T_current2orign)
    # print(T_orign2current @ np.linalg.inv(T_orign2current))
    # print(T_orign2current)
    T_orign2camera = T_lidar2cam @ T_orign2current
    # T_orign2camera = T_imu2cam @ T_orign2current
    coords = project(points, img, M1, T_orign2camera)
    show_with_opencv(img, coords=coords)

