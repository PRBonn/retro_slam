// MIT License
//
// Copyright (c) 2026 Saurabh Gupta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <memory>
#include <string>

// ROS 2
#include <tf2_eigen/tf2_eigen.hpp>

namespace utils {
std::array<double, 36> eigen_hessian_to_covariance_ros_array(
    const Eigen::Matrix<double, 6, 6> &hessian) {
    Eigen::Matrix<double, 6, 6> covariance = hessian.inverse();
    std::array<double, 36> covariance_array;
    std::copy(covariance.data(), covariance.data() + covariance.size(), covariance_array.begin());
    return covariance_array;
}

Eigen::Matrix<double, 6, 6> covariance_ros_array_to_eigen_hessian(
    const std::array<double, 36> &covariance_array) {
    Eigen::Matrix<double, 6, 6> covariance;
    std::copy(covariance_array.begin(), covariance_array.end(), covariance.data());
    return covariance.inverse();
}

Eigen::Isometry3d LookupTransform(const std::string &target_frame,
                                  const std::string &source_frame,
                                  const std::unique_ptr<tf2_ros::Buffer> &tf2_buffer) {
    std::string err_msg;
    if (tf2_buffer->canTransform(target_frame, source_frame, tf2::TimePointZero, &err_msg)) {
        try {
            auto tf = tf2_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf2::transformToEigen(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "%s", ex.what());
        }
    }
    RCLCPP_WARN(rclcpp::get_logger("LookupTransform"), "Failed to find tf. Reason=%s",
                err_msg.c_str());
    return Eigen::Isometry3d::Identity();
}
}  // namespace utils
