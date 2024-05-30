## A script for extracting data from rosbag
 
### Create conda environment
```bash
conda env create -f environment.yml
```

### Parameters 
- input_rosbag: path to the input rosbag file
- output_folder: path to the output folder
- camera_topic: topic name for the camera
- lidar_topic: topic name for the lidar

### How to use
```bash
# Activate conda environment
conda activate process_rosbag
# source envirnonment paramters
source /opt/ros/noetic/setup.zsh 
# Run the script
python process_rosbag.py -i <input_rosbag> -o <output_folder> -t_camera <topic_camera> -t_lidar <topic_lidar>

# Example 1 
python process_rosbag.py \
--input /home/zmxj/code/Datasets/lidar+d455/2024-03-23-13-33-43.bag \
--output /home/zmxj/code/Datasets/lidar+d455/output \
--topic_camera /camera/color/image_raw \
--topic_pointcloud /livox/lidar

# Example 2 
python process_rosbag_infra.py \     
--input /home/zmxj/code/Datasets/20240422cam_infra_lidar/2024-04-22-16-45-53_box.bag \
--output /home/zmxj/code/Datasets/20240422cam_infra_lidar/output \
--topic_camera /camera/color/image_raw \
--topic_pointcloud /livox/lidar \
--infra

# Example 3
python process_rosbag_infra_nopointcloud.py \
--input /home/zmxj/code/Datasets/20240505cam_infra_rgb/d455_infra_rgb.bag \
--output /home/zmxj/code/Datasets/20240505cam_infra_rgb/output \
--topic_camera /camera/color/image_raw \
--topic_pointcloud /livox/lidar \
--infra 


## Example 4 for project lidar pointcloud to camera image
python process_lidar2camera.py

```
