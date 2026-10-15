# MIT License

# Copyright (c) 2026 Saurabh Gupta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

retro_slam_ros_pkg = get_package_share_directory("retro_slam_ros")


def generate_launch_description():
    declare_arguments = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        ),
        DeclareLaunchArgument("log_level", default_value="info", description="Logging level"),
        DeclareLaunchArgument("namespace", default_value="", description="Namespace for the nodes"),
        DeclareLaunchArgument("pointcloud_topic", description="Input point cloud topic"),
        DeclareLaunchArgument(
            "base_frame",
            default_value="",
            description="TF frame attached to the robot body. Leave empty to disable publishing base transforms.",
        ),
        DeclareLaunchArgument(
            "lidar_odom_frame",
            default_value="lidar_odom",
            description="Lidar odometry frame",
        ),
        DeclareLaunchArgument(
            "global_map_frame",
            default_value="lidar_map",
            description="Global map frame",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([retro_slam_ros_pkg, "config", "config.yaml"]),
            description="Path to config file",
        ),
        DeclareLaunchArgument("bag_file", description="Path to rosbag file"),
        DeclareLaunchArgument(
            "read_ahead_queue_size",
            default_value="1000",
            description="Read ahead queue size for rosbag",
        ),
        DeclareLaunchArgument("rviz", default_value="false", description="Launch RViz"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="slam.rviz",
            description="Path to RViz config file",
        ),
    ]

    rviz = LaunchConfiguration("rviz")
    log_level = LaunchConfiguration("log_level")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    base_frame = LaunchConfiguration("base_frame")
    lidar_odom_frame = LaunchConfiguration("lidar_odom_frame")
    global_map_frame = LaunchConfiguration("global_map_frame")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")

    config_file = LaunchConfiguration("config_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    bag_file = LaunchConfiguration("bag_file")
    read_ahead_queue_size = LaunchConfiguration("read_ahead_queue_size")

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([retro_slam_ros_pkg, "launch", "slam.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "log_level": log_level,
            "namespace": namespace,
            "pointcloud_topic": pointcloud_topic,
            "lidar_odom_frame": lidar_odom_frame,
            "global_map_frame": global_map_frame,
            "base_frame": base_frame,
            "config_file": config_file,
            "rviz": rviz,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    bag_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_file,
            "--clock",
            "--read-ahead-queue-size",
            read_ahead_queue_size,
        ],
        output="screen",
    )

    return LaunchDescription([*declare_arguments, slam_launch, bag_node])
