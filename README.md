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
# Run the script
python process_rosbag.py -i <input_rosbag> -o <output_folder> -t_camera <topic_camera> -t_lidar <topic_lidar>

# Example
python process_rosbag.py --input '/home/zmxj/code/Datasets/lidar+d455/2024-03-23-13-33-43.bag' --output /home/zmxj/code/Datasets/lidar+d455/output --topic_camera /camera/color/image_raw --topic_lidar /livox/lidar
```
