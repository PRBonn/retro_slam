// MIT License

// Copyright (c) 2025 Tiziano Guadagnino

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
#include "pose_graph_optimizer.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <memory>

namespace {
static constexpr double kEpsilon = 1e-6;
}
// clang-format off
namespace g2o {
G2O_REGISTER_TYPE(VERTEX_SE3:QUAT, VertexSE3)
G2O_REGISTER_TYPE(EDGE_SE3:QUAT, EdgeSE3)
}  // namespace g2o
// clang-format on

namespace retro_slam {
using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
using AlgorithmType = g2o::OptimizationAlgorithmDogleg;

PoseGraphOptimizer::PoseGraphOptimizer(const int max_iterations, const bool verbose)
    : max_iterations_(max_iterations) {
    graph_ = std::make_unique<g2o::SparseOptimizer>();
    graph_->setVerbose(verbose);

    auto solver =
        new AlgorithmType(std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

    auto terminateAction = new g2o::SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(kEpsilon);
    graph_->addPostIterationAction(terminateAction);
    graph_->setAlgorithm(solver);
}

void PoseGraphOptimizer::FixVariable(const int id) { graph_->vertex(id)->setFixed(true); }

void PoseGraphOptimizer::AddVariable(const int id, const Eigen::Isometry3d &pose) {
    g2o::VertexSE3 *variable = new g2o::VertexSE3();
    variable->setId(id);
    variable->setEstimate(pose);
    graph_->addVertex(variable);
}

void PoseGraphOptimizer::AddFactor(const int id_source,
                                   const int id_target,
                                   const Eigen::Isometry3d &relative_pose,
                                   const Eigen::Matrix6d &information_matrix,
                                   const bool robust_kernel) {
    g2o::EdgeSE3 *factor = new g2o::EdgeSE3();
    factor->setVertex(0, graph_->vertex(id_target));
    factor->setVertex(1, graph_->vertex(id_source));
    factor->setInformation(information_matrix);
    factor->setMeasurement(relative_pose);
    if (robust_kernel) {
        g2o::RobustKernelDCS *rk = new g2o::RobustKernelDCS;
        rk->setDelta(1.0);
        factor->setRobustKernel(rk);
    }
    graph_->addEdge(factor);
}

std::vector<Eigen::Isometry3d> PoseGraphOptimizer::Estimates() const {
    const g2o::HyperGraph::VertexIDMap &variables = graph_->vertices();
    std::vector<Eigen::Isometry3d> poses(variables.size());
    std::for_each(variables.cbegin(), variables.cend(), [&](const auto &id_var) {
        const auto &[id, v] = id_var;
        poses[id] = static_cast<g2o::VertexSE3 *>(v)->estimate();
    });
    return poses;
}

void PoseGraphOptimizer::Optimize() {
    graph_->initializeOptimization();
    graph_->optimize(max_iterations_);
}
}  // namespace retro_slam
