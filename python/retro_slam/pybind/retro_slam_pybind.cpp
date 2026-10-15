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
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "kiss_icp/pipeline/KissICP.hpp"
#include "retro_slam/RetroSlamConfig.h"
#include "retro_slam/retro_slam.hpp"
#include "stl_vector_eigen.h"

namespace py = pybind11;
using namespace py::literals;
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>);

namespace {
py::dict AsDict(const py::handle &obj) {
    if (py::isinstance<py::dict>(obj)) {
        return py::reinterpret_borrow<py::dict>(obj);
    }
    if (py::hasattr(obj, "model_dump")) {
        return py::cast<py::dict>(obj.attr("model_dump")());
    }
    throw std::runtime_error("Expected dict or pydantic model with model_dump().");
}

kiss_icp::pipeline::KISSConfig ParseOdometryConfig(const py::dict &odometry_cfg) {
    kiss_icp::pipeline::KISSConfig odometry;

    const py::dict pre = AsDict(odometry_cfg[py::str("preprocessing")]);
    odometry.max_range = py::cast<double>(pre[py::str("max_range")]);
    odometry.min_range = py::cast<double>(pre[py::str("min_range")]);
    odometry.deskew = py::cast<bool>(pre[py::str("deskew")]);

    const py::dict mapping = AsDict(odometry_cfg[py::str("mapping")]);
    odometry.voxel_size = py::cast<double>(mapping[py::str("voxel_size")]);
    odometry.max_points_per_voxel = py::cast<int>(mapping[py::str("max_points_per_voxel")]);

    const py::dict registration = AsDict(odometry_cfg[py::str("registration")]);
    odometry.max_num_iterations = py::cast<int>(registration[py::str("max_num_iterations")]);
    odometry.convergence_criterion =
        py::cast<double>(registration[py::str("convergence_criterion")]);
    odometry.max_num_threads = py::cast<int>(registration[py::str("max_num_threads")]);

    const py::dict adaptive = AsDict(odometry_cfg[py::str("adaptive_threshold")]);
    odometry.min_motion_th = py::cast<double>(adaptive[py::str("min_motion_th")]);
    odometry.initial_threshold = py::cast<double>(adaptive[py::str("initial_threshold")]);

    return odometry;
}

LoopCloserConfig ParseLoopCloserConfig(const py::dict &loop_closer_cfg) {
    LoopCloserConfig config;
    config.fitness_threshold = py::cast<double>(loop_closer_cfg[py::str("fitness_threshold")]);

    const py::dict detector = AsDict(loop_closer_cfg[py::str("detector")]);
    config.detector.density_map_resolution =
        py::cast<float>(detector[py::str("density_map_resolution")]);
    config.detector.density_threshold = py::cast<float>(detector[py::str("density_threshold")]);
    config.detector.hamming_distance_threshold =
        py::cast<int>(detector[py::str("hamming_distance_threshold")]);
    config.inliers_threshold = py::cast<std::size_t>(detector[py::str("inliers_threshold")]);
    return config;
}

LocalMapperConfig ParseLocalMapperConfig(const py::dict &local_mapper_cfg) {
    LocalMapperConfig config;
    config.submap_voxel_size = py::cast<double>(local_mapper_cfg[py::str("submap_voxel_size")]);
    config.submap_splitting_distance =
        py::cast<double>(local_mapper_cfg[py::str("submap_splitting_distance")]);
    config.localmap_voxel_size = py::cast<double>(local_mapper_cfg[py::str("localmap_voxel_size")]);
    config.localmap_splitting_distance =
        py::cast<double>(local_mapper_cfg[py::str("localmap_splitting_distance")]);
    return config;
}

PoseGraphOptimizerConfig ParsePoseGraphOptimizerConfig(const py::dict &pgo_cfg) {
    PoseGraphOptimizerConfig config;
    config.max_iterations = py::cast<int>(pgo_cfg[py::str("max_iterations")]);
    config.verbose = py::cast<bool>(pgo_cfg[py::str("verbose")]);
    return config;
}
}  // namespace

RetroSlamConfig GetConfigFromPython(const py::handle &cfg_obj) {
    const py::dict cfg = AsDict(cfg_obj);
    RetroSlamConfig config;

    config.out_dir = py::cast<std::string>(cfg[py::str("out_dir")]);
    config.odometry = ParseOdometryConfig(AsDict(cfg[py::str("odometry")]));
    config.loop_closer = ParseLoopCloserConfig(AsDict(cfg[py::str("loop_closer")]));
    config.local_mapper = ParseLocalMapperConfig(AsDict(cfg[py::str("local_mapper")]));
    config.pose_graph_optimizer =
        ParsePoseGraphOptimizerConfig(AsDict(cfg[py::str("pose_graph_optimizer")]));

    return config;
}

PYBIND11_MODULE(retro_slam_pybind, m) {
    using namespace retro_slam;

    auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
        m, "_Vector3dVector", "std::vector<Eigen::Vector3d>",
        py::py_array_to_vectors<Eigen::Vector3d>);
    py::bind_vector<std::vector<Closures>>(m, "ClosuresVector");

    py::class_<Closures>(m, "Closures")
        .def(py::init<int, int>(), "ref_scan_id"_a, "query_scan_id"_a)
        .def_readonly("ref_scan_id", &Closures::ref_scan_id)
        .def_readonly("query_scan_id", &Closures::query_scan_id)
        .def("__iter__",
             [](const Closures &self) {
                 return py::iter(py::make_tuple(self.ref_scan_id, self.query_scan_id));
             })
        .def("__repr__", [](const Closures &self) {
            return "<Closures(ref_scan_id=" + std::to_string(self.ref_scan_id) +
                   ", query_scan_id=" + std::to_string(self.query_scan_id) + ")>";
        });

    m.def("retro_slam_config_from_python", &GetConfigFromPython, "config"_a,
          "Convert a Python dict/pydantic config into C++ RetroSlamConfig");

    py::class_<RetroSLAM>(m, "_RetroSLAM")
        .def(py::init([](const py::object &cfg) {
                 return std::make_unique<RetroSLAM>(GetConfigFromPython(cfg));
             }),
             "config"_a)
        .def("_process_scan", &RetroSLAM::ProcessScan, "scan_points"_a, "scan_timestamps"_a)
        .def("_exit", &RetroSLAM::Exit)
        .def("_poses",
             [](const RetroSLAM &slam) {
                 const auto poses = slam.Poses();
                 py::list result;
                 for (const auto &pose : poses) {
                     result.append(py::cast(pose.matrix()));
                 }
                 return result;
             })
        .def("_keyposes",
             [](const RetroSLAM &slam) {
                 const auto keyposes = slam.Keyposes();
                 py::list result;
                 for (const auto &keypose : keyposes) {
                     result.append(py::cast(keypose.matrix()));
                 }
                 return result;
             })
        .def("_visualization_state",
             [](const RetroSLAM &slam) {
                 const auto state = slam.LatestVisualizationState();
                 return py::make_tuple(
                     py::cast(state.global_trajectory), py::cast(state.local_trajectory),
                     py::cast(state.keyposes), py::cast(state.latest_scan_points),
                     py::cast(state.latest_scan_pose), py::cast(state.loop_closures));
             })
        .def("_write_pose_graph", &RetroSLAM::WritePoseGraph, "filename"_a)
        .def("_loop_closures", &RetroSLAM::LoopClosures);
}
