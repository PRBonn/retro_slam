// MIT License

// Copyright (c) 2026 Saurabh Gupta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

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
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "retro_slam/RetroSlamConfig.h"
#include "retro_slam/retro_slam.hpp"
#include "retro_slam/voxel_map.hpp"

// ROS 2
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <example_interfaces/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace retro_slam_ros {
using NodeSyncPolicy = message_filters::sync_policies::ExactTime<nav_msgs::msg::Odometry,
                                                                 sensor_msgs::msg::PointCloud2>;

struct SubmapData {
    std::vector<Eigen::Isometry3d> local_trajectory;
    std::vector<Eigen::Matrix<double, 6, 6>> hessians;
    std::vector<Eigen::Vector3d> pointcloud;
};

struct LocalMapData {
    int localmap_id;
    std::vector<SubmapData> submaps;
    std::vector<Eigen::Vector3d> pointcloud;
    std::vector<Eigen::Vector3d> voxel_means;
    std::vector<Eigen::Vector3d> voxel_normals;
    Eigen::Isometry3d latest_odom_T_scan;
};

class MappingNode : public rclcpp::Node {
public:
    MappingNode() = delete;
    MappingNode(const MappingNode &) = delete;
    MappingNode &operator=(const MappingNode &) = delete;

    MappingNode(MappingNode &&) = delete;
    MappingNode &operator=(MappingNode &&) = delete;

    explicit MappingNode(const rclcpp::NodeOptions &options);
    ~MappingNode() = default;

private:
    void LoadConfig();
    void InitializeVoxelMaps();
    void InitializeSubscribers();
    void InitializePublishers();
    void InitializeServices();

    void ResetSubmapState();
    void GenerateLocalMapSplit();
    void FinalizeAndQueueRemainingLocalMapData();
    void QueueCurrentLocalMapDataForSLAM();
    void OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan_msg);

    void ProcessLocalMap(const LocalMapData &local_map_data);
    void SLAMWorkerThread();

    void TruncatedGlobalMapPublisher();
    void OdomToMapTF2Publisher();

    void GlobalMapServiceServer(
        const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response);

    void ExitServiceServer(const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
                           std::shared_ptr<example_interfaces::srv::Trigger::Response> response);
    void DumpGlobalMapToDisk();

private:
    std::string map_frame_;
    std::string odom_frame_;
    RetroSlamConfig slam_config_;

    std::unique_ptr<retro_slam::VoxelMap> submap_voxel_grid_;
    std::unique_ptr<retro_slam::VoxelMap> local_map_voxel_grid_;
    std::unique_ptr<retro_slam::VoxelMap> global_map_voxel_grid_;
    double global_map_publisher_radius_threshold_sq_;

    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odometry_subscriber_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
        deskewed_frame_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<NodeSyncPolicy>>
        synchronized_odometry_subscriber_;

    std::mutex slam_core_access_mutex_;
    rclcpp::TimerBase::SharedPtr global_map_publish_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_publisher_;

    std::mutex map_T_odom_mutex_;
    rclcpp::TimerBase::SharedPtr map_T_odom_publish_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> map_T_odom_publisher_;

    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr global_map_saver_service_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr exit_service_;

    std::unique_ptr<retro_slam::RetroSLAM> slam_pipeline_;
    std::thread slam_thread_;

    std::vector<Eigen::Vector3d> latest_deskewed_scan_;

    Eigen::Isometry3d odom_T_submap_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d submap_T_scan_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d odom_T_localmap_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d localmap_T_scan_ = Eigen::Isometry3d::Identity();

    int current_submap_id_ = 0;
    int current_local_map_id_ = 0;

    std::vector<Eigen::Isometry3d> local_trajectory_;
    std::vector<Eigen::Matrix<double, 6, 6>> hessians_;

    LocalMapData local_map_data_;
    Eigen::Isometry3d latest_odom_T_scan_ = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d map_T_odom_ = Eigen::Isometry3d::Identity();
    bool global_map_needs_update_{true};

    std::deque<LocalMapData> local_map_data_queue_;
    std::mutex local_map_queue_mutex_;
    std::condition_variable local_map_cv_;
    bool processing_local_map_{false};

    std::filesystem::path output_dir_;
};
}  // namespace retro_slam_ros
