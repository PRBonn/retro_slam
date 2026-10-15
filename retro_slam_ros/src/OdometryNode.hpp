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
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace retro_slam_ros {

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() = delete;
    OdometryNode(const OdometryNode &) = delete;
    OdometryNode &operator=(const OdometryNode &) = delete;
    OdometryNode(OdometryNode &&) = delete;
    OdometryNode &operator=(OdometryNode &&) = delete;

    explicit OdometryNode(const rclcpp::NodeOptions &options);
    ~OdometryNode() override = default;

private:
    kiss_icp::pipeline::KISSConfig LoadConfig();
    void InitializePublishers();
    void InitializeSubscribers();
    void InitializeTF2();

    void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg);
    void PublishRegistrationData(const std_msgs::msg::Header &header,
                                 const Eigen::Isometry3d &pose,
                                 const Eigen::Matrix<double, 6, 6> &hessian);

private:
    std::string odom_frame_{};
    std::string base_frame_{};

    std::optional<Eigen::Isometry3d> base_frame_T_lidar_frame_;
    std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_pipeline_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_frame_publisher_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::vector<Eigen::Vector3d> latest_scan_;
    std::vector<double> latest_timestamps_;
};

}  // namespace retro_slam_ros
