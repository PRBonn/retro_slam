# Retro-SLAM ROS 2 Wrapper

### Build Instructions

```sh
cd ~/ros2_ws/src/ && git clone https://github.com/PRBonn/retro_slam && cd ~/ros2_ws/ && colcon build --packages-select retro_slam_ros
source ./install/setup.bash
```

### Run on live sensor data

```sh
ros2 launch retro_slam_ros slam.launch.py pointcloud_topic:=<topic_name> rviz:=<True/False>
```

### Run on data from a bagfile

```sh
ros2 launch retro_slam_ros slam_rosbag.launch.py pointcloud_topic:=<topic_name> rviz:=<True/False> bag_file:=<path_to_rosbag>
```

### Dump the current state of global map to disk along with other metadata

This will dump the current state of the global map and optimized poses to the disk in a folder `retro_slam_results/<system_timestamp>`
```
ros2 service call /save_global_map example_interfaces/srv/Trigger \{\}
```

### Exit the mapping node graciously and save global map and metadata to disk 

This will create the final local map and query for loop closures, optimize the pose graph if any loop closures are found, and then dump the global map and optimized poses to the disk in a folder `retro_slam_results/<system_timestamp>`
```
ros2 service call /clean_exit_slam example_interfaces/srv/Trigger \{\}
```

### Configuration

An **example** yaml file is given in config/config.yaml. The file is parsed by default, but any file path can be provided with the `config_file` launch argument.

The ROS-related parameters can be set via command-line when launching the node:
* `base_frame`
* `lidar_odom_frame`
* `global_map_frame`

You can set them like so:
```sh
ros2 launch retro_slam_ros slam.launch.py pointcloud_topic:=<topic_name> <parameter_name>:=<parameter_value>
```

Note that if you set the ROS-related parameters in the yaml configuration file and via command-line, the values in the yaml file will be selected in the end.
