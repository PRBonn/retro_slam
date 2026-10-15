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
#include <vector>

#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"
#include "voxel_map.hpp"

namespace {
constexpr double kKernelScaleFactor = 3.0;
constexpr double kVoxelSize = 0.5;
const double kIcpThreshold = std::sqrt(kKernelScaleFactor) * kVoxelSize;
}  // namespace

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
}

namespace retro_slam {
struct Result {
    Eigen::Isometry3d refined_pose;
    Eigen::Matrix6d hessian;
    double fitness_score;
};

struct Submap {
    Submap(const int submap_id, const int scan_id, const Eigen::Isometry3d &keypose)
        : submap_id_(submap_id), scan_id_(scan_id), keypose_(keypose.matrix()) {}

    void emplace_back(const Eigen::Isometry3d &pose) { traj_.emplace_back(pose.matrix()); }
    Eigen::Isometry3d endpose() const {
        return traj_.size() > 0 ? keypose_ * traj_.back() : keypose_;
    }

    int submap_id_;
    int scan_id_;
    Eigen::Isometry3d keypose_;
    std::vector<Eigen::Isometry3d> traj_;
    std::vector<Eigen::Vector3d> pcd_;
};

struct SubmapGraph {
    SubmapGraph(const kiss_icp::pipeline::KISSConfig &odometry_config);

    const Submap &operator[](const int id) const { return graph_.at(id); }

    std::vector<Eigen::Isometry3d> Keyposes() const;

    Submap &CurrentSubmap() { return graph_.back(); }

    Eigen::Isometry3d CurrentKeypose() const { return graph_.back().keypose_; }

    void EraseCurrentSubmap() {
        graph_.pop_back();
        current_submap_id_--;
    }

    void FinalizeSubmap(const std::vector<Eigen::Isometry3d> &local_trajectory,
                        const std::vector<Eigen::Vector3d> &pointcloud);

    Result RegisterSubmaps(const int source_id,
                           const int target_id,
                           const Eigen::Isometry3d &initial_guess) const;
    int current_submap_id_ = 0;
    std::vector<Submap> graph_;
    kiss_icp::pipeline::KISSConfig odometry_config_;
};
}  // namespace retro_slam
