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
#include <cstddef>
#include <string>

#include "kiss_icp/pipeline/KissICP.hpp"
#include "map_closures/MapClosures.hpp"

struct LoopCloserConfig {
    map_closures::Config detector;
    double fitness_threshold = 0.25;
    size_t inliers_threshold = 5;
};

struct LocalMapperConfig {
    double submap_voxel_size = 0.5;
    double submap_splitting_distance = 25.0;
    double localmap_voxel_size = 0.5;
    double localmap_splitting_distance = 100.0;
};

struct PoseGraphOptimizerConfig {
    int max_iterations = 10;
    bool verbose = false;
};

struct RetroSlamConfig {
    std::string out_dir = "slam_output";
    kiss_icp::pipeline::KISSConfig odometry;
    LoopCloserConfig loop_closer;
    LocalMapperConfig local_mapper;
    PoseGraphOptimizerConfig pose_graph_optimizer;
};
