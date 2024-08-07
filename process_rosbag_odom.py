import os
import cv2
import rosbag
from tqdm import tqdm
from cv_bridge import CvBridge
import argparse


class ExtractBagData(object):
    def __init__(self, bagfile_path, camera_topic, pointcloud_topic, odometry_topic, output_path):
        self.bagfile_path = bagfile_path  # bag 路径
        self.camera_topic = camera_topic  # 相机话题 topic
        self.pointcloud_topic = pointcloud_topic  # 点云话题 topic
        self.odometry_topic = odometry_topic # 里程计话题 topic
        # 在输出路径的名称设置为 output + bag 文件的名称
        output_path = os.path.join(output_path, os.path.basename(bagfile_path).split(".")[0])
        self.output_path = output_path  # 输出的根路径
        self.image_dir = os.path.join(output_path, "images")  # 存放照片的路径
        self.pointcloud_dir = os.path.join(output_path, "pointcloud")  # 存放点云的路径
        self.odometry_dir = os.path.join(output_path, "odometry") # 存放里程计的路径

        # 检查路径是否存在，否则创建提取图片和点云的目录 ./output_path/images  ./output_path/pointcloud
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        if not os.path.exists(self.pointcloud_dir):
            os.makedirs(self.pointcloud_dir)
        if not os.path.exists(self.odometry_dir):
            os.makedirs(self.odometry_dir)

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

    # 从rosbag中将指定topic中的里程计信息提取出来
    # 里程计信息包括：时间戳、位置、姿态

    def bag_to_odometry(self):
        with rosbag.Bag(self.bagfile_path, 'r') as bag:
            # 读取里程计信息
            bag_data_odom = bag.read_messages(self.odometry_topic)
            # 里程计信息的存储路径
            odom_file = os.path.join(self.odometry_dir, "odom.txt")
            # 打开文件，写入里程计信息
            with open(odom_file, 'w') as f:
                # 
                f.write("# timestamp x y z qx qy qz qw\n")
                for index, [topic, msg, t] in enumerate(bag_data_odom, start=0):
                    # 获取时间戳
                    timestamp = str(msg.header.stamp)
                    # 获取位置
                    position = msg.pose.pose.position
                    # 获取姿态
                    orientation = msg.pose.pose.orientation
                    # 写入文件
                    f.write(timestamp + " " + str(position.x) + " " + str(position.y) + " " + str(position.z) + " " + str(orientation.x) + " " + str(orientation.y) + " " + str(orientation.z) + " " + str(orientation.w) + "\n")
                    print("extract odometry id: %s" % (index + 1))


if __name__ == "__main__":

    # 创建解析器
    parser = argparse.ArgumentParser(description='Process rosbag data.')

    # 添加参数
    parser.add_argument('--input', type=str, required=True, help='Absolute path to the bag file.')
    parser.add_argument('--topic_camera', type=str, required=True, help='Camera topic.')
    parser.add_argument('--topic_pointcloud', type=str, required=True, help='Point cloud topic.')
    parser.add_argument('--topic_odometry', type=str, required=True, help='Odometry topic.')
    parser.add_argument('--output', type=str, required=True, help='Root path for output.')

    # 解析参数
    args = parser.parse_args()

    # bagfile_path="/mnt/e/2024-01-13-12-40-28.bag",  # bag 文件的绝对路径
    # camera_topic="/stereo_publisher/color/image",  # 相机 topic
    # pointcloud_topic="/livox/lidar",  # 点云 topic
    # output_path="/mnt/e/output/",  # 输出的根路径

    extract_bag = ExtractBagData(
        bagfile_path=args.input,  # bag 文件的绝对路径
        camera_topic=args.topic_camera,  # 相机 topic
        pointcloud_topic=args.topic_pointcloud,  # 点云 topic
        odometry_topic=args.topic_odometry, # odom topic
        output_path=args.output,  # 输出的根路径
    )

    extract_bag.bag_to_image()  # 提取图片
    extract_bag.bag_to_pointcloud()  # 提取点云
    extract_bag.bag_to_odometry() # 提取里程计定位
