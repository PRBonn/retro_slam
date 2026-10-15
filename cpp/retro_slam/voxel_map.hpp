// MIT License

// Copyright (c) 2025 Tiziano Guadagnino, Benedikt Mersch, Saurabh Gupta, Cyrill
// Stachniss.

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
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <tuple>
#include <unordered_map>
#include <vector>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using Voxel = Eigen::Vector3i;

struct VoxelHash {
    std::size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};

constexpr unsigned int kMaxPointsPerVoxelBlock = 20;
constexpr unsigned int kMinPointsForCovarianceComputation = 10;

namespace retro_slam {

struct VoxelBlock {
    void emplace_back(const Eigen::Vector3d &point);
    size_t size() const { return num_points_; }
    auto cbegin() const { return points_.cbegin(); }
    auto cend() const { return std::next(points_.cbegin(), num_points_); }
    Eigen::Vector3d front() const { return points_.front(); }
    std::array<Eigen::Vector3d, kMaxPointsPerVoxelBlock> points_;
    size_t num_points_ = 0;
};

struct VoxelMap {
    explicit VoxelMap(const double voxel_size);

    void Clear() {
        num_points_ = 0;
        map_.clear();
    }
    bool Empty() const { return map_.empty(); }

    void IntegrateFrame(const Vector3dVector &points, const Eigen::Isometry3d &pose);
    void AddPoints(const Vector3dVector &points);

    Vector3dVector Pointcloud() const;
    Vector3dVector PointsWithinRadiusFromLocation(const Eigen::Vector3d &location,
                                                  const double radius) const;
    std::tuple<Vector3dVector, Vector3dVector> PerVoxelMeanAndNormal();

    size_t num_points_ = 0;
    double voxel_size_;
    double map_resolution_sq_;
    std::unordered_map<Voxel, VoxelBlock, VoxelHash> map_;
};
}  // namespace retro_slam
