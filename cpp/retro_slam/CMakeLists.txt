# MIT License

# Copyright (c) 2026 Saurabh Gupta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
add_library(retro_slam_core STATIC)
target_sources(retro_slam_core PRIVATE voxel_map.cpp submap_graph.cpp loop_closer.cpp pose_graph_optimizer.cpp
                                       retro_slam.cpp)
target_include_directories(retro_slam_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(
  retro_slam_core
  PUBLIC Eigen3::Eigen
         Sophus::Sophus
         kiss_icp_pipeline
         map_closures
         kiss_icp_core
         g2o::core
         g2o::types_slam3d
         g2o::solver_cholmod)
set_global_target_properties(retro_slam_core)
