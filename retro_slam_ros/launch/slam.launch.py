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
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

retro_slam_ros_pkg = FindPackageShare("retro_slam_ros")


def on_node_exit(event, context):
    if event.returncode != 0:
        return [
            LogInfo(msg=f"Node failed with exit code {event.returncode}. Shutting down."),
            EmitEvent(event=Shutdown(reason=f"Node failed with exit code {event.returncode}")),
        ]

    return []


def generate_launch_description():
    declare_arguments = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation time"
        ),
        DeclareLaunchArgument("log_level", default_value="info", description="Logging level"),
        DeclareLaunchArgument("namespace", default_value="", description="Namespace for the nodes"),
        DeclareLaunchArgument("pointcloud_topic", description="Input point cloud topic"),
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
            "base_frame",
            default_value="",
            description="TF frame attached to the robot body. Leave empty to disable publishing base transforms.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([retro_slam_ros_pkg, "config", "config.yaml"]),
            description="Path to config file",
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

    slam_frontend_node = Node(
        package="retro_slam_ros",
        executable="odometry_node",
        name="odometry_node",
        namespace=namespace,
        output="screen",
        remappings=[("pointcloud_topic", pointcloud_topic)],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "lidar_odom_frame": lidar_odom_frame,
                "base_frame": base_frame,
            },
            config_file,
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    slam_backend_node = Node(
        package="retro_slam_ros",
        executable="mapping_node",
        name="mapping_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "lidar_odom_frame": lidar_odom_frame,
                "global_map_frame": global_map_frame,
            },
            {"use_sim_time": use_sim_time},
            config_file,
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    shutdown_on_failure = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=slam_backend_node, on_exit=on_node_exit)
    )

    rviz_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([retro_slam_ros_pkg, "launch", "visualization.launch.py"])
        ),
        launch_arguments={
            "rviz_config_file": rviz_config_file,
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            *declare_arguments,
            slam_frontend_node,
            slam_backend_node,
            shutdown_on_failure,
            rviz_launch_include,
        ]
    )
