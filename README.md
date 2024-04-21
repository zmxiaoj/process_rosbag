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
python process_rosbag.py
```
