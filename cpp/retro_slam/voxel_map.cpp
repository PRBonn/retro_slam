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
#include "voxel_map.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>

namespace {
const double kSqrt3 = std::sqrt(3.0);

inline Voxel ToVoxelCoordinates(const Eigen::Vector3d &point, const double voxel_size) {
    return Voxel(static_cast<int>(std::floor(point.x() / voxel_size)),
                 static_cast<int>(std::floor(point.y() / voxel_size)),
                 static_cast<int>(std::floor(point.z() / voxel_size)));
}

inline Eigen::Vector3d ToWorldCoordinates(const Voxel &voxel, const double voxel_size) {
    return Eigen::Vector3d(static_cast<double>(voxel.x()) * voxel_size,
                           static_cast<double>(voxel.y()) * voxel_size,
                           static_cast<double>(voxel.z()) * voxel_size) +
           Eigen::Vector3d::Constant(voxel_size * 0.5);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> ComputeMeanAndNormal(
    const retro_slam::VoxelBlock &voxel_block) {
    const double num_points = static_cast<double>(voxel_block.size());
    const Eigen::Vector3d mean =
        std::reduce(voxel_block.cbegin(), voxel_block.cend(), Eigen::Vector3d().setZero()) /
        num_points;

    const Eigen::Matrix3d covariance =
        std::transform_reduce(voxel_block.cbegin(), voxel_block.cend(), Eigen::Matrix3d().setZero(),
                              std::plus<Eigen::Matrix3d>(),
                              [&mean](const Eigen::Vector3d &point) {
                                  Eigen::Vector3d centered = point - mean;
                                  Eigen::Matrix3d outer_product = centered * centered.transpose();
                                  return outer_product;
                              }) /
        (num_points - 1);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    const Eigen::Vector3d normal = solver.eigenvectors().col(0);
    return {mean, normal};
}
}  // namespace

namespace retro_slam {

void VoxelBlock::emplace_back(const Eigen::Vector3d &p) {
    if (num_points_ < kMaxPointsPerVoxelBlock) {
        points_[num_points_] = p;
        num_points_++;
    }
}

VoxelMap::VoxelMap(const double voxel_size)
    : voxel_size_(voxel_size),
      map_resolution_sq_((voxel_size * voxel_size) / static_cast<double>(kMaxPointsPerVoxelBlock)) {
}

Vector3dVector VoxelMap::Pointcloud() const {
    Vector3dVector points;
    points.reserve(num_points_);
    for (auto it = map_.begin(); it != map_.end(); ++it) {
        const VoxelBlock &voxel_block = it->second;
        points.insert(points.end(), voxel_block.cbegin(), voxel_block.cend());
    }
    return points;
}

Vector3dVector VoxelMap::PointsWithinRadiusFromLocation(const Eigen::Vector3d &location,
                                                        const double radius) const {
    const double half_diagonal = 0.5 * kSqrt3 * voxel_size_;
    const auto max_distance2 = (radius + half_diagonal) * (radius + half_diagonal);
    Vector3dVector nearby_points;
    nearby_points.reserve(num_points_);
    for (auto it = map_.begin(); it != map_.end(); ++it) {
        if ((ToWorldCoordinates(it->first, voxel_size_) - location).squaredNorm() <
            (max_distance2)) {
            const VoxelBlock &voxel_block = it->second;
            nearby_points.insert(nearby_points.end(), voxel_block.cbegin(), voxel_block.cend());
        }
    }
    return nearby_points;
}

std::tuple<Vector3dVector, Vector3dVector> VoxelMap::PerVoxelMeanAndNormal() {
    Vector3dVector voxel_means;
    voxel_means.reserve(map_.size());
    Vector3dVector voxel_normals;
    voxel_normals.reserve(map_.size());
    for (auto it = map_.begin(); it != map_.end(); ++it) {
        VoxelBlock &voxel_block = it->second;
        if (voxel_block.size() >= kMinPointsForCovarianceComputation) {
            const auto &[mean, normal] = ComputeMeanAndNormal(voxel_block);
            voxel_means.emplace_back(mean);
            voxel_normals.emplace_back(normal);
        }
    }
    return {std::move(voxel_means), std::move(voxel_normals)};
}

void VoxelMap::IntegrateFrame(const Vector3dVector &points, const Eigen::Isometry3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    AddPoints(points_transformed);
}

void VoxelMap::AddPoints(const Vector3dVector &points) {
    for (const auto &point : points) {
        {
            const Voxel voxel = ToVoxelCoordinates(point, voxel_size_);
            VoxelBlock &voxel_block = map_[voxel];
            if (voxel_block.size() == kMaxPointsPerVoxelBlock ||
                std::any_of(voxel_block.cbegin(), voxel_block.cend(),
                            [&](const Eigen::Vector3d &voxel_point) {
                                return (voxel_point - point).squaredNorm() < map_resolution_sq_;
                            })) {
                continue;
            }
            voxel_block.emplace_back(point);
            num_points_++;
        }
    }
}
}  // namespace retro_slam
