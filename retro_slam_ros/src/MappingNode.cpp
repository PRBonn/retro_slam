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
#include "MappingNode.hpp"

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include "kiss_icp/pipeline/KissICP.hpp"
#include "pointcloud_utils.hpp"
#include "retro_slam/retro_slam.hpp"
#include "retro_slam/runtime_profiler.hpp"
#include "retro_slam/voxel_map.hpp"
#include "utils.hpp"

// ROS 2 headers
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <rmw/qos_profiles.h>

#include <example_interfaces/srv/trigger.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using pointcloud_utils::default_schema_xyz;
using pointcloud_utils::ros_pointcloud2_to_vector_of_eigen;
using pointcloud_utils::save_pointcloud_as_ply;
using pointcloud_utils::vector_of_eigen_to_ros_pointcloud2;
using utils::covariance_ros_array_to_eigen_hessian;

namespace {
constexpr size_t kExpectedSubmapsPerLocalMap = 8;

void save_poses_kitti_format(const std::filesystem::path &file_path,
                             const std::vector<Eigen::Isometry3d> &poses) {
    std::ofstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing poses in KITTI format");
    }
    for (const auto &pose : poses) {
        const auto translation = pose.translation();
        const auto rotation = pose.rotation();
        file << std::fixed << std::setprecision(6) << rotation(0, 0) << " " << rotation(0, 1) << " "
             << rotation(0, 2) << " " << translation(0) << " " << rotation(1, 0) << " "
             << rotation(1, 1) << " " << rotation(1, 2) << " " << translation(1) << " "
             << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << " "
             << translation(2) << "\n";
    }
}

void save_loop_closure_indices(const std::filesystem::path &file_path,
                               const std::vector<retro_slam::Closures> &loop_closures) {
    std::ofstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing loop closure indices");
    }
    for (const auto &closure : loop_closures) {
        file << closure.ref_scan_id << "," << closure.query_scan_id << "\n";
    }
}
}  // namespace

namespace retro_slam_ros {
MappingNode::MappingNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("mapping_node", options) {
    map_frame_ = declare_parameter<std::string>("global_map_frame", map_frame_);
    odom_frame_ = declare_parameter<std::string>("lidar_odom_frame", odom_frame_);

    LoadConfig();
    InitializeVoxelMaps();
    InitializeSubscribers();
    InitializePublishers();
    InitializeServices();

    global_map_publisher_radius_threshold_sq_ =
        1.5 * 1.5 * slam_config_.odometry.max_range * slam_config_.odometry.max_range;
    slam_pipeline_ = std::make_unique<retro_slam::RetroSLAM>(slam_config_);
    slam_thread_ = std::thread(&MappingNode::SLAMWorkerThread, this);

    local_map_data_ = LocalMapData();
    local_map_data_.localmap_id = current_local_map_id_;
    local_map_data_.submaps.reserve(kExpectedSubmapsPerLocalMap);

    const auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm = *std::localtime(&time);
    std::ostringstream oss;
    oss << std::put_time(&tm, "retro_slam_results/%Y-%m-%d_%H-%M-%S");
    output_dir_ = oss.str();
    std::filesystem::create_directories(output_dir_);
    // Create a symlink to output_dir_
    const std::filesystem::path symlink_path = "retro_slam_results/latest";
    if (std::filesystem::is_symlink(std::filesystem::symlink_status(symlink_path))) {
        std::filesystem::remove(symlink_path);
    }
    std::filesystem::create_directory_symlink(output_dir_.filename(), symlink_path);

    const std::string profiler_path = output_dir_ / "runtime_profile_backend.json";
    retro_slam::profiler::RuntimeProfiler::Instance().SetDefaultPath(profiler_path);

    RCLCPP_INFO(get_logger(), "Retro-SLAM ROS 2 local mapper node initialized");
}

void MappingNode::LoadConfig() {
    LocalMapperConfig local_mapper_config;
    local_mapper_config.submap_voxel_size = declare_parameter<double>(
        "local_mapper.submap_voxel_size", local_mapper_config.submap_voxel_size);
    local_mapper_config.submap_splitting_distance = declare_parameter<double>(
        "local_mapper.submap_splitting_distance", local_mapper_config.submap_splitting_distance);
    local_mapper_config.localmap_voxel_size = declare_parameter<double>(
        "local_mapper.localmap_voxel_size", local_mapper_config.localmap_voxel_size);
    local_mapper_config.localmap_splitting_distance =
        declare_parameter<double>("local_mapper.localmap_splitting_distance",
                                  local_mapper_config.localmap_splitting_distance);

    LoopCloserConfig loop_closer_config;
    loop_closer_config.detector = map_closures::Config();
    loop_closer_config.inliers_threshold = declare_parameter<int>(
        "loop_closer.inliers_threshold", loop_closer_config.inliers_threshold);
    loop_closer_config.fitness_threshold = declare_parameter<double>(
        "loop_closer.fitness_threshold", loop_closer_config.fitness_threshold);

    PoseGraphOptimizerConfig pgo_config;
    pgo_config.max_iterations =
        declare_parameter<int>("pose_graph_optimizer.max_iterations", pgo_config.max_iterations);
    pgo_config.verbose =
        declare_parameter<bool>("pose_graph_optimizer.verbose", pgo_config.verbose);

    kiss_icp::pipeline::KISSConfig kiss_icp_config;
    kiss_icp_config.max_range =
        declare_parameter<double>("odometry.data.max_range", kiss_icp_config.max_range);
    kiss_icp_config.max_num_iterations = declare_parameter<int>(
        "odometry.registration.max_num_iterations", kiss_icp_config.max_num_iterations);
    kiss_icp_config.convergence_criterion = declare_parameter<double>(
        "odometry.registration.convergence_criterion", kiss_icp_config.convergence_criterion);
    kiss_icp_config.max_num_threads = declare_parameter<int>(
        "odometry.registration.max_num_threads", kiss_icp_config.max_num_threads);

    slam_config_.local_mapper = local_mapper_config;
    slam_config_.loop_closer = loop_closer_config;
    slam_config_.pose_graph_optimizer = pgo_config;
    slam_config_.odometry = kiss_icp_config;
}

void MappingNode::InitializeVoxelMaps() {
    submap_voxel_grid_ =
        std::make_unique<retro_slam::VoxelMap>(slam_config_.local_mapper.submap_voxel_size);
    local_map_voxel_grid_ =
        std::make_unique<retro_slam::VoxelMap>(slam_config_.local_mapper.localmap_voxel_size);
    global_map_voxel_grid_ =
        std::make_unique<retro_slam::VoxelMap>(slam_config_.odometry.voxel_size);
}

void MappingNode::InitializeSubscribers() {
    rclcpp::SubscriptionOptions options;
    options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    deskewed_frame_subscriber_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, "deskewed_frame", rclcpp::SensorDataQoS().keep_last(20).get_rmw_qos_profile(),
            options);
    odometry_subscriber_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
        this, "lidar_odometry", rclcpp::SensorDataQoS().keep_last(20).get_rmw_qos_profile(),
        options);

    synchronized_odometry_subscriber_ =
        std::make_shared<message_filters::Synchronizer<NodeSyncPolicy>>(
            NodeSyncPolicy(50), *odometry_subscriber_, *deskewed_frame_subscriber_);
    synchronized_odometry_subscriber_->registerCallback(std::bind(
        &MappingNode::OdometryCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void MappingNode::InitializePublishers() {
    global_map_publish_timer_ = create_wall_timer(
        std::chrono::seconds(5), std::bind(&MappingNode::TruncatedGlobalMapPublisher, this));
    global_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "global_map", rclcpp::QoS(1).reliable().transient_local());

    map_T_odom_publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&MappingNode::OdomToMapTF2Publisher, this));
    map_T_odom_publisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void MappingNode::InitializeServices() {
    const auto service_callback_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    global_map_saver_service_ = create_service<example_interfaces::srv::Trigger>(
        "save_global_map",
        std::bind(&MappingNode::GlobalMapServiceServer, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_default);
    exit_service_ = create_service<example_interfaces::srv::Trigger>(
        "clean_exit_slam",
        std::bind(&MappingNode::ExitServiceServer, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_default);
}

void MappingNode::ResetSubmapState() {
    RCLCPP_DEBUG(get_logger(), "Generating new submap: current_submap_id=%d", current_submap_id_);

    const auto retained_points = submap_voxel_grid_->PointsWithinRadiusFromLocation(
        submap_T_scan_.translation(), slam_config_.odometry.max_range);
    submap_voxel_grid_->Clear();
    submap_voxel_grid_->IntegrateFrame(retained_points, submap_T_scan_.inverse());

    local_trajectory_.clear();
    local_trajectory_.emplace_back(Eigen::Isometry3d::Identity());
    hessians_.clear();
    hessians_.emplace_back(Eigen::Matrix<double, 6, 6>::Identity());

    current_submap_id_++;
    odom_T_submap_ = latest_odom_T_scan_;
}

void MappingNode::GenerateLocalMapSplit() {
    RCLCPP_DEBUG(get_logger(), "Generating new local map: localmap_id=%d",
                 local_map_data_.localmap_id);

    local_map_data_.pointcloud = local_map_voxel_grid_->Pointcloud();
    auto [voxel_means, voxel_normals] = local_map_voxel_grid_->PerVoxelMeanAndNormal();
    local_map_data_.voxel_means = std::move(voxel_means);
    local_map_data_.voxel_normals = std::move(voxel_normals);
    local_map_data_.latest_odom_T_scan = latest_odom_T_scan_;

    const auto retained_points = local_map_voxel_grid_->PointsWithinRadiusFromLocation(
        localmap_T_scan_.translation(), slam_config_.odometry.max_range);
    local_map_voxel_grid_->Clear();
    local_map_voxel_grid_->IntegrateFrame(retained_points, localmap_T_scan_.inverse());

    odom_T_localmap_ = latest_odom_T_scan_;
}

void MappingNode::QueueCurrentLocalMapDataForSLAM() {
    std::unique_lock<std::mutex> local_map_queue_lock(local_map_queue_mutex_);
    local_map_data_queue_.emplace_back(std::move(local_map_data_));
    local_map_queue_lock.unlock();
    local_map_cv_.notify_one();

    current_local_map_id_++;
    local_map_data_ = LocalMapData();
    local_map_data_.localmap_id = current_local_map_id_;
    local_map_data_.submaps.reserve(kExpectedSubmapsPerLocalMap);
}

void MappingNode::FinalizeAndQueueRemainingLocalMapData() {
    if (!submap_voxel_grid_->Empty() && !local_trajectory_.empty()) {
        local_map_data_.submaps.emplace_back(local_trajectory_, hessians_,
                                             submap_voxel_grid_->Pointcloud());
    }

    if (local_map_data_.submaps.empty() && local_map_voxel_grid_->Empty()) {
        return;
    }

    GenerateLocalMapSplit();
    QueueCurrentLocalMapDataForSLAM();
}

void MappingNode::OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                                   const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan_msg) {
    retro_slam::profiler::ScopedTimer timer_total("retro_slam.MappingNode.OdometryCallback.total");
    Eigen::fromMsg(odom_msg->pose.pose, latest_odom_T_scan_);
    ros_pointcloud2_to_vector_of_eigen(*scan_msg, default_schema_xyz, latest_deskewed_scan_);

    localmap_T_scan_ = odom_T_localmap_.inverse() * latest_odom_T_scan_;
    submap_T_scan_ = odom_T_submap_.inverse() * latest_odom_T_scan_;
    local_map_voxel_grid_->IntegrateFrame(latest_deskewed_scan_, localmap_T_scan_);
    submap_voxel_grid_->IntegrateFrame(latest_deskewed_scan_, submap_T_scan_);

    local_trajectory_.emplace_back(submap_T_scan_);
    hessians_.emplace_back(covariance_ros_array_to_eigen_hessian(odom_msg->pose.covariance));

    const bool split_local_map = localmap_T_scan_.translation().norm() >
                                 slam_config_.local_mapper.localmap_splitting_distance;
    const bool split_submap =
        submap_T_scan_.translation().norm() > slam_config_.local_mapper.submap_splitting_distance;
    if (split_submap || split_local_map) {
        local_map_data_.submaps.emplace_back(local_trajectory_, hessians_,
                                             submap_voxel_grid_->Pointcloud());
        ResetSubmapState();
    }
    if (split_local_map) {
        GenerateLocalMapSplit();
        QueueCurrentLocalMapDataForSLAM();
    }
    timer_total.end();
}

void MappingNode::SLAMWorkerThread() {
    while (true) {
        LocalMapData local_map_data;
        {
            std::unique_lock<std::mutex> lock(local_map_queue_mutex_);
            local_map_cv_.wait(lock, [&] { return !local_map_data_queue_.empty(); });
            local_map_data = std::move(local_map_data_queue_.front());
            local_map_data_queue_.pop_front();
            processing_local_map_ = true;
            RCLCPP_DEBUG(get_logger(), "LocalMap queue size %ld", local_map_data_queue_.size());
        }
        ProcessLocalMap(local_map_data);
        {
            std::lock_guard<std::mutex> lock(local_map_queue_mutex_);
            processing_local_map_ = false;
        }
        local_map_cv_.notify_all();
    }
}

void MappingNode::ProcessLocalMap(const LocalMapData &local_map_data) {
    retro_slam::profiler::ScopedTimer timer_total("retro_slam.MappingNode.ProcessLocalMap.total");
    const int localmap_id = local_map_data.localmap_id;
    RCLCPP_INFO(get_logger(), "Processing Local Map %d", localmap_id);
    {
        std::lock_guard<std::mutex> lock(slam_core_access_mutex_);
        for (const SubmapData &submap_data : local_map_data.submaps) {
            slam_pipeline_->FinalizeSubmap(submap_data.local_trajectory, submap_data.hessians,
                                           submap_data.pointcloud);
        }
        const bool found_closures = slam_pipeline_->ComputeClosures(
            local_map_data.pointcloud, local_map_data.voxel_means, local_map_data.voxel_normals);
        if (found_closures) {
            RCLCPP_INFO(get_logger(), "Loop closure detected for Local Map %d", localmap_id);
            std::lock_guard<std::mutex> lock(map_T_odom_mutex_);
            map_T_odom_ =
                slam_pipeline_->CurrentKeypose() * local_map_data.latest_odom_T_scan.inverse();
        }
        global_map_needs_update_ = true;
    }
    timer_total.end();
    RCLCPP_INFO(get_logger(), "Processed Local Map %d", localmap_id);
}

void MappingNode::OdomToMapTF2Publisher() {
    std::lock_guard<std::mutex> lock(map_T_odom_mutex_);
    geometry_msgs::msg::TransformStamped odom_to_map_tf = tf2::eigenToTransform(map_T_odom_);
    odom_to_map_tf.header.stamp = get_clock()->now();
    odom_to_map_tf.header.frame_id = map_frame_;
    odom_to_map_tf.child_frame_id = odom_frame_;
    map_T_odom_publisher_->sendTransform(odom_to_map_tf);
}

void MappingNode::TruncatedGlobalMapPublisher() {
    std::unique_lock<std::mutex> slam_lock(slam_core_access_mutex_);
    if (global_map_needs_update_) {
        global_map_voxel_grid_->Clear();
        const auto &submaps = slam_pipeline_->Submaps();
        const auto &current_keypose = slam_pipeline_->CurrentKeypose();
        for (auto it = submaps.cbegin(); it != submaps.cend() - 1; ++it) {
            const auto &submap = *it;
            const Eigen::Vector3d delta =
                submap.keypose_.translation() - current_keypose.translation();
            if (delta.squaredNorm() < global_map_publisher_radius_threshold_sq_) {
                global_map_voxel_grid_->IntegrateFrame(submap.pcd_, submap.keypose_);
            }
        }
        const auto pointcloud = global_map_voxel_grid_->Pointcloud();
        global_map_needs_update_ = false;
        slam_lock.unlock();

        std_msgs::msg::Header header;
        header.stamp = get_clock()->now();
        header.frame_id = map_frame_;
        const sensor_msgs::msg::PointCloud2 global_map_msg =
            vector_of_eigen_to_ros_pointcloud2(header, pointcloud);
        global_map_publisher_->publish(global_map_msg);
    }
}

void MappingNode::GlobalMapServiceServer(
    const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
    std::shared_ptr<example_interfaces::srv::Trigger::Response> response) {
    RCLCPP_INFO(get_logger(), "Global map saver service called.");
    (void)request;
    DumpGlobalMapToDisk();
    response->success = true;
    response->message = "Generating global map and saving to disk.";
}

void MappingNode::DumpGlobalMapToDisk() {
    std::unique_lock<std::mutex> slam_lock(slam_core_access_mutex_);
    global_map_voxel_grid_->Clear();
    const auto &submaps = slam_pipeline_->Submaps();
    for (auto it = submaps.cbegin(); it != submaps.cend() - 1; ++it) {
        const auto &submap = *it;
        global_map_voxel_grid_->IntegrateFrame(submap.pcd_, submap.keypose_);
    }
    const auto pointcloud = global_map_voxel_grid_->Pointcloud();
    slam_pipeline_->WritePoseGraph(output_dir_ / "pose_graph.g2o");
    save_poses_kitti_format(output_dir_ / "poses_kitti.txt", slam_pipeline_->Poses());
    save_loop_closure_indices(output_dir_ / "loop_closures.txt", slam_pipeline_->LoopClosures());
    slam_lock.unlock();
    save_pointcloud_as_ply(output_dir_ / "global_map.ply", pointcloud);
    RCLCPP_INFO(get_logger(), "Global map saved to disk at %s", output_dir_.c_str());
}

void MappingNode::ExitServiceServer(
    const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
    std::shared_ptr<example_interfaces::srv::Trigger::Response> response) {
    RCLCPP_INFO(get_logger(), "Exiting mapping node. Finalizing and saving global map to disk.");
    (void)request;
    FinalizeAndQueueRemainingLocalMapData();
    {
        std::unique_lock<std::mutex> lock(local_map_queue_mutex_);
        local_map_cv_.wait(
            lock, [this] { return local_map_data_queue_.empty() && !processing_local_map_; });
    }
    DumpGlobalMapToDisk();
    response->success = true;
    response->message = "Generating global map and saving to disk.";
}
}  // namespace retro_slam_ros

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<retro_slam_ros::MappingNode>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    retro_slam::profiler::RuntimeProfiler::Instance().FlushToDefault();
    rclcpp::shutdown();
    return 0;
}
