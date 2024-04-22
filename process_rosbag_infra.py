import os
import cv2
import rosbag
from tqdm import tqdm
from cv_bridge import CvBridge
import argparse


class ExtractBagData(object):
    def __init__(self, bagfile_path, camera_topic, pointcloud_topic, output_path):
        self.bagfile_path = bagfile_path  # bag 路径
        self.camera_topic = camera_topic  # 相机话题 topic
        self.camera_infra1_topic = "/camera/infra1/image_rect_raw"
        self.camera_infra2_topic = "/camera/infra2/image_rect_raw"
        self.pointcloud_topic = pointcloud_topic  # 点云话题 topic
        # 在输出路径的名称设置为 output + bag 文件的名称
        output_path = os.path.join(output_path, os.path.basename(bagfile_path).split(".")[0])
        self.output_path = output_path  # 输出的根路径
        self.image_dir = os.path.join(output_path, "images")  # 存放照片的路径
        self.image_infra_dir = os.path.join(output_path, "infra_images")  # 存放点云的路径
        self.pointcloud_dir = os.path.join(output_path, "pointcloud")  # 存放点云的路径

        # 检查路径是否存在，否则创建提取图片和点云的目录 ./output_path/images  ./output_path/pointcloud
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        if not os.path.exists(self.pointcloud_dir):
            os.makedirs(self.pointcloud_dir)
        if not os.path.exists(self.image_infra_dir):
            os.makedirs(self.image_infra_dir)

    def bag_to_image(self):
        """ 提取图片
        """
        bag = rosbag.Bag(self.bagfile_path, "r")  # 读取 bag
        bridge = CvBridge()  # 用于将图像消息转为图片
        bag_data_imgs = bag.read_messages(self.camera_topic)  # 读取图像消息

        pbar = tqdm(bag_data_imgs)
        for index, [topic, msg, t] in enumerate(pbar, start=0):
            pbar.set_description("extract image id: %s" % (index + 1))
            # 消息转为图片
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            # 获取时间戳 
            # msg.header.stamp表示话题发布的时间戳，t表示消息接收的时间戳   
            timestamp = str(msg.header.stamp)
            # print("\ntimestamp: ", timestamp)
            t = str(t.to_nsec())
            # print("t: ", t)
            # 存储图片 .bmp .png .jpg
            # cv2.imwrite(os.path.join(self.image_dir, str(index) + ".png"), cv_image)
            cv2.imwrite(os.path.join(self.image_dir, t + ".png"), cv_image)
            # index += 1
    
    def bag_to_infra_image(self):
        """ 提取图片
        """
        bag = rosbag.Bag(self.bagfile_path, "r")  # 读取 bag
        bridge = CvBridge()  # 用于将图像消息转为图片
        bag_data_infra_imgs1 = bag.read_messages(self.camera_infra1_topic)  # 读取图像消息
        bag_data_infra_imgs2 = bag.read_messages(self.camera_infra2_topic)

        pbar1 = tqdm(bag_data_infra_imgs1)
        for index, [topic, msg, t] in enumerate(pbar1, start=0):
            pbar1.set_description("extract image id: %s" % (index + 1))
            # 消息转为图片
            cv_image = bridge.imgmsg_to_cv2(msg, "mono8")
            # 获取时间戳 
            t = str(t.to_nsec())
            # print("t: ", t)
            # 存储图片 .bmp .png .jpg
            cv2.imwrite(os.path.join(self.image_infra_dir, t + "_1.png"), cv_image)

        pbar2 = tqdm(bag_data_infra_imgs2)
        for index, [topic, msg, t] in enumerate(pbar2, start=0):
            pbar2.set_description("extract image id: %s" % (index + 1))
            # 消息转为图片
            cv_image = bridge.imgmsg_to_cv2(msg, "mono8")
            # 获取时间戳 
            t = str(t.to_nsec())
            # print("t: ", t)
            # 存储图片 .bmp .png .jpg
            cv2.imwrite(os.path.join(self.image_infra_dir, t + "_2.png"), cv_image)

    def bag_to_pointcloud(self):
        """ 提取点云
        - 提取点云数据为 pcd 后缀文件，默认提取以时间戳命名
        - 提取命令：rosrun pcl_ros bag_to_pcd result.bag /velodyne_points ./pointcloud
        """
        # 启动roscore
        os.system("roscore &")
        cmd = "rosrun pcl_ros bag_to_pcd %s %s %s" % (
            self.bagfile_path,
            self.pointcloud_topic,
            self.pointcloud_dir,
        )
        os.system(cmd)

        # 读取提取的 pcd 点云数据，把文件名修改为按照顺序索引名
        pcd_files_list = os.listdir(self.pointcloud_dir)
        # 因为提取的 pcd 是以时间戳命名的，但是存放到列表中并不是按照时间戳从小到大排列，这里对时间戳进行重新排序
        pcd_files_list_sorted = sorted(pcd_files_list, reverse=False)

        pbar = tqdm(pcd_files_list_sorted)
        for index, pcd_file in enumerate(pbar, start=0):
            pbar.set_description("extract poindcloud id: %s" % (index + 1))
            # 
            os.rename(
                os.path.join(self.pointcloud_dir, pcd_file),
                os.path.join(self.pointcloud_dir, pcd_file.replace(".", "", 1)),
            )
            # 去掉文件名第一个"."
            pcd_file = pcd_file.replace(".", "", 1)
            # 将 pcd 文件转换为 ply 文件
            ply_file = pcd_file.split(".")[0] + ".ply"
            cmd = "pcl_pcd2ply %s %s" % (
                os.path.join(self.pointcloud_dir, pcd_file),
                os.path.join(self.pointcloud_dir, ply_file),
            )
            os.system(cmd)
            print("pcd_file name: ", pcd_file)
            print("ply_file name: ", ply_file)



if __name__ == "__main__":

    # 创建解析器
    parser = argparse.ArgumentParser(description='Process rosbag data.')

    # 添加参数
    parser.add_argument('--input', type=str, required=True, help='Absolute path to the bag file.')
    parser.add_argument('--topic_camera', type=str, required=True, help='Camera topic.')
    parser.add_argument('--topic_pointcloud', type=str, required=True, help='Point cloud topic.')
    parser.add_argument('--output', type=str, required=True, help='Root path for output.')
    parser.add_argument('--infra', action='store_true',  help='Infra1 topic.')

    # 解析参数
    args = parser.parse_args()

    extract_bag = ExtractBagData(
        bagfile_path=args.input,  # bag 文件的绝对路径
        camera_topic=args.topic_camera,  # 相机 topic
        pointcloud_topic=args.topic_pointcloud,  # 点云 topic
        output_path=args.output,  # 输出的根路径
    )

    extract_bag.bag_to_image()  # 提取图片
    if args.infra:
        extract_bag.bag_to_infra_image()  # 提取infra图片
    extract_bag.bag_to_pointcloud()  # 提取点云

