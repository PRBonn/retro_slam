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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

retro_slam_ros_pkg = FindPackageShare("retro_slam_ros")


def generate_launch_description():
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the nodes"
    )
    namespace = LaunchConfiguration("namespace")

    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config_file", default_value="slam.rviz", description="Path to RViz config file"
    )
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([retro_slam_ros_pkg, "rviz", rviz_config_file]),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [declare_namespace_arg, declare_use_sim_time_arg, declare_rviz_config_arg, rviz_node]
    )
