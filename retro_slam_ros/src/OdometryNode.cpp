// MIT License
//
// Copyright (c) 2026 Saurabh Gupta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "OdometryNode.hpp"

#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

#include "kiss_icp/pipeline/KissICP.hpp"
#include "pointcloud_utils.hpp"
#include "utils.hpp"

// ROS 2 headers
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

using pointcloud_utils::get_timestamps;
using pointcloud_utils::parse_pointcloud2_schema;
using pointcloud_utils::ros_pointcloud2_to_vector_of_eigen;
using pointcloud_utils::vector_of_eigen_to_ros_pointcloud2;
using utils::eigen_hessian_to_covariance_ros_array;
using utils::LookupTransform;

namespace retro_slam_ros {
OdometryNode::OdometryNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    odom_frame_ = declare_parameter<std::string>("lidar_odom_frame", odom_frame_);
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);

    kiss_icp_pipeline_ = std::make_unique<kiss_icp::pipeline::KissICP>(LoadConfig());

    InitializeTF2();
    InitializePublishers();
    InitializeSubscribers();

    RCLCPP_INFO(this->get_logger(), "ROS 2 odometry node initialized");
}

kiss_icp::pipeline::KISSConfig OdometryNode::LoadConfig() {
    kiss_icp::pipeline::KISSConfig config;
    config.max_range = declare_parameter<double>("odometry.data.max_range", config.max_range);
    config.min_range = declare_parameter<double>("odometry.data.min_range", config.min_range);
    config.deskew = declare_parameter<bool>("odometry.data.deskew", config.deskew);
    config.voxel_size =
        declare_parameter<double>("odometry.mapping.voxel_size", config.max_range / 100.0);
    config.max_points_per_voxel = declare_parameter<int>("odometry.mapping.max_points_per_voxel",
                                                         config.max_points_per_voxel);
    config.initial_threshold = declare_parameter<double>(
        "odometry.adaptive_threshold.initial_threshold", config.initial_threshold);
    config.min_motion_th = declare_parameter<double>("odometry.adaptive_threshold.min_motion_th",
                                                     config.min_motion_th);
    config.max_num_iterations = declare_parameter<int>("odometry.registration.max_num_iterations",
                                                       config.max_num_iterations);
    config.convergence_criterion = declare_parameter<double>(
        "odometry.registration.convergence_criterion", config.convergence_criterion);
    config.max_num_threads =
        declare_parameter<int>("odometry.registration.max_num_threads", config.max_num_threads);
    if (config.max_range < config.min_range) {
        RCLCPP_WARN(get_logger(),
                    "[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
        config.min_range = 0.0;
    }
    RCLCPP_INFO(get_logger(), "Voxel size %.2f m, range %.1f-%.1f m", config.voxel_size,
                config.min_range, config.max_range);
    return config;
}

void OdometryNode::InitializePublishers() {
    lidar_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        "lidar_odometry", rclcpp::QoS(10).reliable().durability_volatile());
    deskewed_frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "deskewed_frame", rclcpp::SensorDataQoS().keep_last(10));
}

void OdometryNode::InitializeSubscribers() {
    rclcpp::SubscriptionOptions subscriber_options;
    subscriber_options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto lidar_data_qos = rclcpp::SensorDataQoS().keep_last(10);
    lidar_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", lidar_data_qos,
        std::bind(&OdometryNode::RegisterFrame, this, std::placeholders::_1), subscriber_options);
}

void OdometryNode::InitializeTF2() {
    tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
}

void OdometryNode::RegisterFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg) {
    const std::string lidar_frame_id = pointcloud_msg->header.frame_id;
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == lidar_frame_id);
    const auto moving_frame = egocentric_estimation ? lidar_frame_id : base_frame_;

    if (moving_frame != lidar_frame_id && !base_frame_T_lidar_frame_) {
        RCLCPP_INFO_ONCE(get_logger(), "Looking up static transform from %s to %s",
                         lidar_frame_id.c_str(), moving_frame.c_str());
        base_frame_T_lidar_frame_ =
            LookupTransform(moving_frame, pointcloud_msg->header.frame_id, tf2_buffer_);
    }

    const auto pointfield_schema = parse_pointcloud2_schema(*pointcloud_msg);
    ros_pointcloud2_to_vector_of_eigen(*pointcloud_msg, pointfield_schema, latest_scan_,
                                       base_frame_T_lidar_frame_);
    get_timestamps(*pointcloud_msg, pointfield_schema, latest_timestamps_);
    const auto &[deskewed_frame, _] =
        kiss_icp_pipeline_->RegisterFrame(latest_scan_, latest_timestamps_);

    std_msgs::msg::Header header;
    header.stamp = pointcloud_msg->header.stamp;
    header.frame_id = moving_frame;
    const Eigen::Isometry3d pose = Eigen::Isometry3d(kiss_icp_pipeline_->pose().matrix());
    const Eigen::Matrix<double, 6, 6> &hessian = kiss_icp_pipeline_->hessian();
    PublishRegistrationData(header, pose, hessian);
    deskewed_frame_publisher_->publish(vector_of_eigen_to_ros_pointcloud2(header, deskewed_frame));
}

void OdometryNode::PublishRegistrationData(const std_msgs::msg::Header &header,
                                           const Eigen::Isometry3d &pose,
                                           const Eigen::Matrix<double, 6, 6> &hessian) {
    geometry_msgs::msg::TransformStamped tf2_msg = tf2::eigenToTransform(pose);
    tf2_msg.header.stamp = header.stamp;
    tf2_msg.header.frame_id = odom_frame_;
    tf2_msg.child_frame_id = header.frame_id;
    tf2_broadcaster_->sendTransform(tf2_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = header.frame_id;
    odom_msg.pose.pose = Eigen::toMsg(pose);
    odom_msg.pose.covariance = eigen_hessian_to_covariance_ros_array(hessian);
    lidar_odom_publisher_->publish(odom_msg);
}
}  // namespace retro_slam_ros

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto odometry_node = std::make_shared<retro_slam_ros::OdometryNode>(rclcpp::NodeOptions());
    executor.add_node(odometry_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
