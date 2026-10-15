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
#include "retro_slam.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

#include "RetroSlamConfig.h"
#include "submap_graph.hpp"

namespace {
std::vector<Eigen::Isometry3d> ComputeLocalmapTrajectory(
    const std::span<retro_slam::Submap> &submaps) {
    const size_t num_poses =
        std::transform_reduce(submaps.begin(), submaps.end(), 0, std::plus<>(),
                              [](const auto &submap) { return submap.traj_.size(); });
    std::vector<Eigen::Isometry3d> localmap_trajectory;
    localmap_trajectory.reserve(num_poses);
    for (const auto &submap : submaps) {
        const auto &keypose = submap.keypose_;
        for (const auto &pose : submap.traj_) {
            localmap_trajectory.emplace_back(keypose * pose);
        }
    }
    return localmap_trajectory;
}

inline double ComputeHausdorffDistance(const std::vector<Eigen::Isometry3d> &source_traj,
                                       const std::vector<Eigen::Isometry3d> &target_traj,
                                       const double max_distance_threshold) {
    double max_distance_threshold_sq = max_distance_threshold * max_distance_threshold;
    double max_distance = 0.0;

    // Check distance from source to target
    for (const auto &pose_src : source_traj) {
        double min_distance = std::numeric_limits<double>::max();
        for (const auto &pose_tgt : target_traj) {
            min_distance = std::min(
                min_distance, (pose_src.translation() - pose_tgt.translation()).squaredNorm());
        }
        if (min_distance > max_distance_threshold_sq) {
            return std::numeric_limits<double>::max();
        }
        max_distance = std::max(max_distance, min_distance);
    }

    // Check distance from target to source
    for (const auto &pose_tgt : target_traj) {
        double min_distance = std::numeric_limits<double>::max();
        for (const auto &pose_src : source_traj) {
            min_distance = std::min(
                min_distance, (pose_src.translation() - pose_tgt.translation()).squaredNorm());
        }
        if (min_distance > max_distance_threshold_sq) {
            return std::numeric_limits<double>::max();
        }
        max_distance = std::max(max_distance, min_distance);
    }
    return std::sqrt(max_distance);
}
}  // namespace

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
}  // namespace Eigen

using retro_slam::Submap;
using Vector3dVector = std::vector<Eigen::Vector3d>;

namespace retro_slam {
RetroSLAM::RetroSLAM(const RetroSlamConfig &config)
    : config_(config),
      hausdorff_distance_threshold_(config.odometry.max_range),
      odometry_pipeline_(config.odometry),
      submap_graph_(config.odometry),
      closer_(config.loop_closer),
      optimizer_(config.pose_graph_optimizer.max_iterations, config.pose_graph_optimizer.verbose),
      submap_voxel_grid_(config.local_mapper.submap_voxel_size),
      local_map_voxel_grid_(config.local_mapper.localmap_voxel_size) {
    optimizer_.AddVariable(0, Eigen::Isometry3d::Identity());
    optimizer_.FixVariable(0);
}

void RetroSLAM::ProcessScan(const Vector3dVector &scan_points,
                            const std::vector<double> &scan_timestamps) {
    const auto &[deskewed_frame, _] =
        odometry_pipeline_.RegisterFrame(scan_points, scan_timestamps);
    const Eigen::Matrix6d &hessian = odometry_pipeline_.hessian();
    const Eigen::Isometry3d scan_to_odom_pose(odometry_pipeline_.pose().matrix());
    slam_state_.localmap_T_scan = slam_state_.odom_T_localmap.inverse() * scan_to_odom_pose;
    slam_state_.submap_T_scan = slam_state_.odom_T_submap.inverse() * scan_to_odom_pose;

    local_map_voxel_grid_.IntegrateFrame(deskewed_frame, slam_state_.localmap_T_scan);
    submap_voxel_grid_.IntegrateFrame(deskewed_frame, slam_state_.submap_T_scan);

    local_trajectory_.emplace_back(slam_state_.submap_T_scan);
    hessians_.emplace_back(hessian);

    latest_scan_points_ = deskewed_frame;
    latest_scan_pose_ = submap_graph_.CurrentKeypose() * slam_state_.submap_T_scan;

    const bool split_local_map = slam_state_.localmap_T_scan.translation().norm() >
                                 config_.local_mapper.localmap_splitting_distance;
    const bool split_submap = slam_state_.submap_T_scan.translation().norm() >
                              config_.local_mapper.submap_splitting_distance;
    if (split_submap || split_local_map) {
        FinalizeSubmap(local_trajectory_, hessians_, submap_voxel_grid_.Pointcloud());
        ResetSubmapState();
        slam_state_.odom_T_submap = scan_to_odom_pose;
    }
    if (split_local_map) {
        const auto &[pointcloud, voxel_means, voxel_normals] = GenerateLocalMapSplit();
        ComputeClosures(pointcloud, voxel_means, voxel_normals);
        ResetLocalMapState();
        slam_state_.odom_T_localmap = scan_to_odom_pose;
    }
}

void RetroSLAM::FinalizeSubmap(const std::vector<Eigen::Isometry3d> &trajectory,
                               const std::vector<Eigen::Matrix<double, 6, 6>> &hessians,
                               const std::vector<Eigen::Vector3d> &pointcloud) {
    submap_ids_in_local_map_.emplace_back(submap_graph_.current_submap_id_);
    const Eigen::Isometry3d current_keypose = submap_graph_.CurrentKeypose();
    for (int id = 1; id < static_cast<int>(trajectory.size()); ++id) {
        const int current_scan_id = submap_graph_.CurrentSubmap().scan_id_ + id;
        const Eigen::Isometry3d &previous_local_pose = trajectory[id - 1];
        const Eigen::Isometry3d &current_local_pose = trajectory[id];
        optimizer_.AddVariable(current_scan_id, current_keypose * current_local_pose);
        optimizer_.AddFactor(current_scan_id, current_scan_id - 1,
                             previous_local_pose.inverse() * current_local_pose, hessians[id],
                             false);
    }
    submap_graph_.FinalizeSubmap(trajectory, pointcloud);
}

void RetroSLAM::ResetSubmapState() {
    const Vector3dVector retained_points = submap_voxel_grid_.PointsWithinRadiusFromLocation(
        slam_state_.submap_T_scan.translation(), config_.odometry.max_range);
    submap_voxel_grid_.Clear();
    submap_voxel_grid_.IntegrateFrame(retained_points, slam_state_.submap_T_scan.inverse());
    local_trajectory_.erase(local_trajectory_.begin() + 1, local_trajectory_.end());
    hessians_.erase(hessians_.begin() + 1, hessians_.end());
}

std::tuple<Vector3dVector, Vector3dVector, Vector3dVector> RetroSLAM::GenerateLocalMapSplit() {
    Vector3dVector pointcloud = local_map_voxel_grid_.Pointcloud();
    auto [voxel_means, voxel_normals] = local_map_voxel_grid_.PerVoxelMeanAndNormal();
    return {std::move(pointcloud), std::move(voxel_means), std::move(voxel_normals)};
}

void RetroSLAM::ResetLocalMapState() {
    const Vector3dVector retained_points = local_map_voxel_grid_.PointsWithinRadiusFromLocation(
        slam_state_.localmap_T_scan.translation(), config_.odometry.max_range);
    local_map_voxel_grid_.Clear();
    local_map_voxel_grid_.IntegrateFrame(retained_points, slam_state_.localmap_T_scan.inverse());
}

bool RetroSLAM::ComputeClosures(const Vector3dVector &query_points,
                                const Vector3dVector &voxel_means,
                                const Vector3dVector &voxel_normals) {
    slam_state_.submaps_per_localmap.emplace_back(submap_ids_in_local_map_);
    submap_ids_in_local_map_.clear();
    const auto loop_closure_result = closer_.Compute(query_points, voxel_means, voxel_normals);
    if (loop_closure_result.success) {
        return ComputeLoopClosureConstraints(loop_closure_result.ref_id,
                                             loop_closure_result.query_id,
                                             loop_closure_result.pose_constraint);
    }
    return false;
}

bool RetroSLAM::ComputeLoopClosureConstraints(const int src_localmap_id,
                                              const int tgt_localmap_id,
                                              const Eigen::Isometry3d &pose_constraint) {
    const auto &src_submap_ids = slam_state_.submaps_per_localmap[src_localmap_id];
    std::span<Submap> src_submaps =
        std::span{submap_graph_.graph_}.subspan(src_submap_ids[0], src_submap_ids.size());

    const auto &tgt_submap_ids = slam_state_.submaps_per_localmap[tgt_localmap_id];
    const std::span<Submap> tgt_submaps =
        std::span{submap_graph_.graph_}.subspan(tgt_submap_ids[0], tgt_submap_ids.size());

    const Eigen::Isometry3d rel_pose_constraint =
        tgt_submaps[0].keypose_ * pose_constraint * src_submaps[0].keypose_.inverse();
    const auto candidates =
        FindSubmapClosureCandidates(src_submaps, tgt_submaps, rel_pose_constraint);

    const bool closure_inserted = RegisterSubmapClosures(candidates);

    if (closure_inserted) {
        OptimizePoseGraph();
        if (slam_state_.last_closure_query_id != -1) {
            ComputeGraphTraversalConstraints(tgt_localmap_id, src_localmap_id);
        }
        slam_state_.last_closure_query_id = tgt_localmap_id;
        slam_state_.last_closure_ref_id = src_localmap_id;
    }
    return closure_inserted;
}

void RetroSLAM::ComputeGraphTraversalConstraints(const int current_localmap_query_id,
                                                 const int current_localmap_ref_id) {
    bool closure_inserted = false;
    for (int tgt_localmap_id = slam_state_.last_closure_query_id + 1;
         tgt_localmap_id < current_localmap_query_id; ++tgt_localmap_id) {
        const auto &tgt_submap_ids = slam_state_.submaps_per_localmap[tgt_localmap_id];
        const std::span<Submap> tgt_submaps =
            std::span{submap_graph_.graph_}.subspan(tgt_submap_ids[0], tgt_submap_ids.size());

        const auto target_localmap_trajectory = ComputeLocalmapTrajectory(tgt_submaps);
        for (int src_localmap_id =
                 std::min(current_localmap_ref_id, slam_state_.last_closure_ref_id) + 1;
             src_localmap_id < std::max(current_localmap_ref_id, slam_state_.last_closure_ref_id);
             ++src_localmap_id) {
            const auto &src_submap_ids = slam_state_.submaps_per_localmap[src_localmap_id];
            const std::span<Submap> src_submaps =
                std::span{submap_graph_.graph_}.subspan(src_submap_ids[0], src_submap_ids.size());

            if ((tgt_submaps[0].keypose_.translation() - src_submaps[0].keypose_.translation())
                    .squaredNorm() >
                (4 * config_.odometry.max_range * config_.odometry.max_range)) {
                continue;
            }

            const auto source_localmap_trajectory = ComputeLocalmapTrajectory(src_submaps);
            const double hausdorff_distance = ComputeHausdorffDistance(
                source_localmap_trajectory, target_localmap_trajectory, config_.odometry.max_range);
            if (hausdorff_distance <= config_.odometry.max_range) {
                const auto candidates = FindSubmapClosureCandidates(src_submaps, tgt_submaps,
                                                                    Eigen::Isometry3d::Identity());
                closure_inserted = RegisterSubmapClosures(candidates) || closure_inserted;
            }
        }
    }
    if (closure_inserted) OptimizePoseGraph();
}

std::vector<SubmapClosureCandidate> RetroSLAM::FindSubmapClosureCandidates(
    const std::span<Submap> &src_submaps,
    const std::span<Submap> &tgt_submaps,
    const Eigen::Isometry3d &rel_pose_constraint) {
    std::vector<SubmapClosureCandidate> candidates;
    candidates.reserve(src_submaps.size());
    for (const Submap &src_submap : src_submaps) {
        double lowest_hausdorff_distance = hausdorff_distance_threshold_;
        std::optional<int> tgt_idx = std::nullopt;
        const Eigen::Isometry3d tgt_T_src = rel_pose_constraint * src_submap.keypose_;
        std::vector<Eigen::Isometry3d> transformed_src_traj;
        transformed_src_traj.reserve(src_submap.traj_.size());
        for (const auto &pose : src_submap.traj_) {
            transformed_src_traj.emplace_back(tgt_T_src * pose);
        }

        for (int idx = 0; idx < static_cast<int>(tgt_submaps.size()); ++idx) {
            const Submap &tgt_submap = tgt_submaps[idx];
            const Eigen::Isometry3d tgt_keypose = tgt_submap.keypose_;
            std::vector<Eigen::Isometry3d> transformed_tgt_traj;
            transformed_tgt_traj.reserve(tgt_submap.traj_.size());
            for (const auto &pose : tgt_submap.traj_) {
                transformed_tgt_traj.emplace_back(tgt_keypose * pose);
            }
            const double hausdorff_distance = ComputeHausdorffDistance(
                transformed_src_traj, transformed_tgt_traj, hausdorff_distance_threshold_);
            if (hausdorff_distance <= lowest_hausdorff_distance) {
                lowest_hausdorff_distance = hausdorff_distance;
                tgt_idx = idx;
            }
        }
        if (tgt_idx.has_value()) {
            const Submap &matched_tgt_submap = tgt_submaps[tgt_idx.value()];
            const Eigen::Isometry3d initial_guess =
                matched_tgt_submap.keypose_.inverse() * rel_pose_constraint * src_submap.keypose_;
            candidates.emplace_back(src_submap.submap_id_, matched_tgt_submap.submap_id_,
                                    src_submap.scan_id_, matched_tgt_submap.scan_id_,
                                    initial_guess);
        }
    }
    return candidates;
}

bool RetroSLAM::RegisterSubmapClosures(const std::vector<SubmapClosureCandidate> &candidates) {
    bool closure_inserted = false;
    for (const auto &candidate : candidates) {
        const auto registration_result = submap_graph_.RegisterSubmaps(
            candidate.src_id, candidate.tgt_id, candidate.initial_guess);
        if (registration_result.fitness_score > config_.loop_closer.fitness_threshold) {
            closure_inserted = true;
            loop_closures_.emplace_back(candidate.src_id, candidate.tgt_id);
            optimizer_.AddFactor(candidate.src_scan_id, candidate.tgt_scan_id,
                                 registration_result.refined_pose, registration_result.hessian,
                                 false);
        }
    }
    return closure_inserted;
}

void RetroSLAM::OptimizePoseGraph() {
    optimizer_.Optimize();
    const auto &estimates = optimizer_.Estimates();

    for (auto it = submap_graph_.graph_.begin(); it != submap_graph_.graph_.end() - 1; ++it) {
        Submap &submap = *it;
        const int submap_scan_id = submap.scan_id_;
        submap.keypose_ = estimates[submap_scan_id];
        const Eigen::Isometry3d inv_keypose = submap.keypose_.inverse();
        for (int i = 0; i < static_cast<int>(submap.traj_.size()); ++i) {
            submap.traj_[i] = inv_keypose * estimates[submap_scan_id + i];
        }
    }

    Submap &current_submap = submap_graph_.CurrentSubmap();
    current_submap.keypose_ = estimates[current_submap.scan_id_];
}

void RetroSLAM::Exit() {
    if (local_trajectory_.size() > 1) {
        FinalizeSubmap(local_trajectory_, hessians_, submap_voxel_grid_.Pointcloud());
        const auto &[pointcloud, voxel_means, voxel_normals] = GenerateLocalMapSplit();
        ComputeClosures(pointcloud, voxel_means, voxel_normals);
    }
    submap_graph_.EraseCurrentSubmap();
}
}  // namespace retro_slam
