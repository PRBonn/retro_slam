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
#include "submap_graph.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <sophus/se3.hpp>
#include <vector>

#include "kiss_icp/core/VoxelHashMap.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"
#include "voxel_map.hpp"

namespace retro_slam {
SubmapGraph::SubmapGraph(const kiss_icp::pipeline::KISSConfig &odometry_config)
    : current_submap_id_(0), odometry_config_(odometry_config) {
    Submap first_submap(0, 0, Eigen::Isometry3d::Identity());
    graph_.emplace_back(first_submap);
}

std::vector<Eigen::Isometry3d> SubmapGraph::Keyposes() const {
    std::vector<Eigen::Isometry3d> poses(graph_.size());
    std::transform(graph_.cbegin(), graph_.cend(), poses.begin(),
                   [](const auto &submap) { return submap.keypose_; });
    return poses;
}

void SubmapGraph::FinalizeSubmap(const std::vector<Eigen::Isometry3d> &local_trajectory,
                                 const std::vector<Eigen::Vector3d> &pointcloud) {
    Submap &submap = CurrentSubmap();
    submap.pcd_ = pointcloud;
    submap.traj_ = local_trajectory;

    current_submap_id_++;
    const int scan_id = submap.scan_id_ + static_cast<int>(submap.traj_.size()) - 1;
    const Eigen::Isometry3d new_keypose = submap.endpose();
    graph_.emplace_back(current_submap_id_, scan_id, new_keypose);
}

Result SubmapGraph::RegisterSubmaps(const int source_id,
                                    const int target_id,
                                    const Eigen::Isometry3d &initial_guess) const {
    kiss_icp::Registration registration_pipeline(odometry_config_.max_num_iterations,
                                                 odometry_config_.convergence_criterion, 0);
    const Vector3dVector &source_points = graph_[source_id].pcd_;
    const Vector3dVector &target_points = graph_[target_id].pcd_;
    kiss_icp::VoxelHashMap target_map(kVoxelSize, odometry_config_.max_range);
    target_map.AddPoints(target_points);
    const Sophus::SE3d pose = registration_pipeline.AlignPointsToMap(
        source_points, target_map, Sophus::SE3d(initial_guess.matrix()), kIcpThreshold,
        kKernelScaleFactor);
    const Eigen::Matrix6d hessian =
        registration_pipeline.GetHessian(source_points, target_map, pose, kIcpThreshold);
    return {Eigen::Isometry3d(pose.matrix()), hessian, registration_pipeline.fitness_};
}
}  // namespace retro_slam
