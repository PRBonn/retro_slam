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
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include "RetroSlamConfig.h"
#include "kiss_icp/pipeline/KissICP.hpp"
#include "loop_closer.hpp"
#include "pose_graph_optimizer.hpp"
#include "submap_graph.hpp"
#include "voxel_map.hpp"

namespace {
struct SubmapClosureCandidate {
    int src_id;
    int tgt_id;
    int src_scan_id;
    int tgt_scan_id;
    Eigen::Isometry3d initial_guess;

    SubmapClosureCandidate(const int src_id,
                           const int tgt_id,
                           const int src_scan_id,
                           const int tgt_scan_id,
                           const Eigen::Isometry3d &initial_guess)
        : src_id(src_id),
          tgt_id(tgt_id),
          src_scan_id(src_scan_id),
          tgt_scan_id(tgt_scan_id),
          initial_guess(initial_guess) {}
};
}  // namespace

namespace retro_slam {
struct SlamState {
    int last_closure_query_id = -1;
    int last_closure_ref_id = -1;
    Eigen::Isometry3d odom_T_localmap = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d localmap_T_scan = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d odom_T_submap = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d submap_T_scan = Eigen::Isometry3d::Identity();

    std::vector<std::vector<int>> submaps_per_localmap;
};

struct Closures {
    int ref_scan_id;
    int query_scan_id;
    Closures(const int ref_id, const int query_id) : ref_scan_id(ref_id), query_scan_id(query_id) {}
};

struct VisualizationState {
    std::vector<Eigen::Isometry3d> global_trajectory;
    std::vector<Eigen::Isometry3d> local_trajectory;
    std::vector<Eigen::Isometry3d> keyposes;
    std::vector<Eigen::Vector3d> latest_scan_points;
    Eigen::Isometry3d latest_scan_pose = Eigen::Isometry3d::Identity();
    std::vector<Closures> loop_closures;
};

class RetroSLAM {
public:
    explicit RetroSLAM(const RetroSlamConfig &config);
    void ProcessScan(const std::vector<Eigen::Vector3d> &scan_points,
                     const std::vector<double> &scan_timestamps);
    void FinalizeSubmap(const std::vector<Eigen::Isometry3d> &trajectory,
                        const std::vector<Eigen::Matrix<double, 6, 6>> &hessians,
                        const std::vector<Eigen::Vector3d> &pointcloud);
    bool ComputeClosures(const std::vector<Eigen::Vector3d> &query_points,
                         const std::vector<Eigen::Vector3d> &voxel_means,
                         const std::vector<Eigen::Vector3d> &voxel_normals);
    void WritePoseGraph(const std::string &filename) const { optimizer_.WriteGraph(filename); }
    void Exit();

    std::vector<Eigen::Isometry3d> Poses() const { return optimizer_.Estimates(); }
    std::vector<Eigen::Isometry3d> Keyposes() const { return submap_graph_.Keyposes(); }
    Eigen::Isometry3d CurrentKeypose() const { return submap_graph_.CurrentKeypose(); }
    const std::vector<Submap> &Submaps() const { return submap_graph_.graph_; }
    const std::vector<Closures> &LoopClosures() const { return loop_closures_; }

    VisualizationState LatestVisualizationState() const {
        VisualizationState state;
        state.global_trajectory = optimizer_.Estimates();
        state.local_trajectory = local_trajectory_;
        state.keyposes = submap_graph_.Keyposes();
        state.latest_scan_points = latest_scan_points_;
        state.latest_scan_pose = latest_scan_pose_;
        state.loop_closures = loop_closures_;
        return state;
    }

private:
    void ResetSubmapState();
    std::tuple<std::vector<Eigen::Vector3d>,
               std::vector<Eigen::Vector3d>,
               std::vector<Eigen::Vector3d>>
    GenerateLocalMapSplit();
    void ResetLocalMapState();

private:
    bool ComputeLoopClosureConstraints(const int src_localmap_id,
                                       const int tgt_localmap_id,
                                       const Eigen::Isometry3d &pose_constraint);

    void ComputeGraphTraversalConstraints(const int current_localmap_query_id,
                                          const int current_localmap_ref_id);

    std::vector<SubmapClosureCandidate> FindSubmapClosureCandidates(
        const std::span<Submap> &src_submaps,
        const std::span<Submap> &tgt_submaps,
        const Eigen::Isometry3d &rel_pose_constraint);

    bool RegisterSubmapClosures(const std::vector<SubmapClosureCandidate> &results);
    bool RegisterSubmapTraversals(const std::vector<SubmapClosureCandidate> &results);

    void OptimizePoseGraph();

private:
    RetroSlamConfig config_;

    SlamState slam_state_{};
    std::vector<Eigen::Isometry3d> local_trajectory_;
    std::vector<Eigen::Matrix<double, 6, 6>> hessians_;
    std::vector<int> submap_ids_in_local_map_;
    double hausdorff_distance_threshold_ = 0.0;

    kiss_icp::pipeline::KissICP odometry_pipeline_;
    SubmapGraph submap_graph_;
    LoopCloser closer_;
    PoseGraphOptimizer optimizer_;

    VoxelMap submap_voxel_grid_;
    VoxelMap local_map_voxel_grid_;
    std::vector<Eigen::Vector3d> latest_scan_points_;
    Eigen::Isometry3d latest_scan_pose_ = Eigen::Isometry3d::Identity();

    std::vector<Closures> loop_closures_;
};
}  // namespace retro_slam
