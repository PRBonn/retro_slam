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
#include "loop_closer.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <vector>

#include "RetroSlamConfig.h"
#include "map_closures/MapClosures.hpp"

namespace retro_slam {
LoopCloser::LoopCloser(const LoopCloserConfig &closer_config)

    : query_id_(-1), closer_config_(closer_config), detector_(closer_config.detector) {}

ClosureResult LoopCloser::Compute(const std::vector<Eigen::Vector3d> &points,
                                  const std::vector<Eigen::Vector3d> &voxel_means,
                                  const std::vector<Eigen::Vector3d> &voxel_normals) {
    ++query_id_;
    const map_closures::ClosureCandidate closure =
        detector_.GetBestClosure(query_id_, points, voxel_means, voxel_normals);
    if (closure.number_of_inliers > closer_config_.inliers_threshold) {
        return {closure.source_id, query_id_, Eigen::Isometry3d(closure.pose), true};
    }
    return {-1, query_id_, Eigen::Isometry3d::Identity(), false};
}
}  // namespace retro_slam
