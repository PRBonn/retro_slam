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

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// ROS 2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

namespace pointcloud_utils {
using Vector3dVector = std::vector<Eigen::Vector3d>;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using std_msgs::msg::Header;

struct PointCloudSchema {
    PointField x;
    PointField y;
    PointField z;
    std::optional<PointField> nx;
    std::optional<PointField> ny;
    std::optional<PointField> nz;
    std::optional<PointField> timestamp;
};

inline PointField create_point_field(const std::string &name,
                                     const uint32_t offset,
                                     const uint8_t datatype,
                                     const uint32_t count) {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
}

const PointCloudSchema default_schema_xyz = {create_point_field("x", 0, PointField::FLOAT32, 1),
                                             create_point_field("y", 4, PointField::FLOAT32, 1),
                                             create_point_field("z", 8, PointField::FLOAT32, 1),
                                             std::nullopt,
                                             std::nullopt,
                                             std::nullopt,
                                             std::nullopt};
const PointCloudSchema default_schema_xyz_normal{
    create_point_field("x", 0, PointField::FLOAT32, 1),
    create_point_field("y", 4, PointField::FLOAT32, 1),
    create_point_field("z", 8, PointField::FLOAT32, 1),
    create_point_field("nx", 12, PointField::FLOAT32, 1),
    create_point_field("ny", 16, PointField::FLOAT32, 1),
    create_point_field("nz", 20, PointField::FLOAT32, 1),
    std::nullopt};

void save_pointcloud_as_ply(const std::string &filename, const Vector3dVector &points) {
    std::ofstream ply_file(filename, std::ios_base::out | std::ios::binary);
    ply_file << "ply\n";
    ply_file << "format binary_little_endian 1.0\n";
    ply_file << "element vertex " << points.size() << "\n";
    ply_file << "property double x\n";
    ply_file << "property double y\n";
    ply_file << "property double z\n";
    ply_file << "end_header\n";

    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        ply_file.write(reinterpret_cast<const char *>(point.data()),
                       sizeof(point[0]) * point.size());
    });
    ply_file.close();
}

PointCloud2 create_pointcloud2_msg(const Header &header,
                                   const size_t n_points,
                                   const bool has_normals = false) {
    PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    cloud_msg.header = header;
    cloud_msg.fields.clear();
    int offset = 0;
    offset = addPointField(cloud_msg, "x", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "y", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "z", 1, PointField::FLOAT32, offset);
    if (has_normals) {
        offset = addPointField(cloud_msg, "nx", 1, PointField::FLOAT32, offset);
        offset = addPointField(cloud_msg, "ny", 1, PointField::FLOAT32, offset);
        offset = addPointField(cloud_msg, "nz", 1, PointField::FLOAT32, offset);
    }
    offset += sizeOfPointField(PointField::FLOAT32);
    // Resize the point cloud accordingly
    cloud_msg.point_step = offset;
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
    modifier.resize(n_points);
    return cloud_msg;
}

void fill_pointcloud2_msg(const Vector3dVector &points, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
        const Eigen::Vector3d &point = points[i];
        *msg_x = static_cast<float>(point.x());
        *msg_y = static_cast<float>(point.y());
        *msg_z = static_cast<float>(point.z());
    }
}

void fill_pointcloud2_msg(const Vector3dVector &points,
                          const Vector3dVector &normals,
                          PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> msg_nx(msg, "nx");
    sensor_msgs::PointCloud2Iterator<float> msg_ny(msg, "ny");
    sensor_msgs::PointCloud2Iterator<float> msg_nz(msg, "nz");
    for (size_t i = 0; i < points.size();
         i++, ++msg_x, ++msg_y, ++msg_z, ++msg_nx, ++msg_ny, ++msg_nz) {
        const Eigen::Vector3d &point = points[i];
        const Eigen::Vector3d &normal = normals[i];
        *msg_x = static_cast<float>(point.x());
        *msg_y = static_cast<float>(point.y());
        *msg_z = static_cast<float>(point.z());
        *msg_nx = static_cast<float>(normal.x());
        *msg_ny = static_cast<float>(normal.y());
        *msg_nz = static_cast<float>(normal.z());
    }
}

PointCloud2 vector_of_eigen_to_ros_pointcloud2(const Header &header, const Vector3dVector &points) {
    const bool has_normals = false;
    PointCloud2 msg = create_pointcloud2_msg(header, points.size(), has_normals);
    fill_pointcloud2_msg(points, msg);
    return msg;
}

PointCloud2 vector_of_eigen_to_ros_pointcloud2(const Header &header,
                                               const Vector3dVector &points,
                                               const Vector3dVector &normals) {
    const bool has_normals = true;
    PointCloud2 msg = create_pointcloud2_msg(header, points.size(), has_normals);
    fill_pointcloud2_msg(points, normals, msg);
    return msg;
}

inline size_t coordinate_field_size(const PointField &field) {
    switch (field.datatype) {
        case PointField::FLOAT32:
            return sizeof(float);
        case PointField::FLOAT64:
            return sizeof(double);
        default:
            throw std::runtime_error("PointCloud2 field '" + field.name +
                                     "' must be FLOAT32 or FLOAT64");
    }
}

inline size_t timestamp_field_size(const PointField &field) {
    switch (field.datatype) {
        case PointField::UINT32:
            return sizeof(uint32_t);
        case PointField::FLOAT32:
            return sizeof(float);
        case PointField::FLOAT64:
            return sizeof(double);
        default:
            throw std::runtime_error("PointCloud2 timestamp field '" + field.name +
                                     "' has an unsupported datatype");
    }
}

inline void validate_pointcloud_layout(const PointCloud2 &msg) {
    if (msg.width == 0 || msg.height == 0) return;
    if (msg.point_step == 0) {
        throw std::runtime_error("PointCloud2 has points but point_step is zero");
    }
    const size_t min_row_step = static_cast<size_t>(msg.width) * msg.point_step;
    if (msg.row_step < min_row_step || msg.row_step > msg.data.size() / msg.height) {
        throw std::runtime_error("PointCloud2 has an invalid row_step or data length");
    }
}

inline void validate_field_fits_points(const PointCloud2 &msg,
                                       const PointField &field,
                                       const size_t field_size) {
    if (field.count != 1 || field.offset > msg.point_step ||
        field_size > msg.point_step - field.offset) {
        throw std::runtime_error("PointCloud2 field '" + field.name +
                                 "' does not fit within point_step");
    }
}

inline PointCloudSchema parse_pointcloud2_schema(const PointCloud2 &msg,
                                                 const bool require_normals = false) {
    std::optional<PointField> x;
    std::optional<PointField> y;
    std::optional<PointField> z;
    PointCloudSchema schema;
    for (const PointField &field : msg.fields) {
        if (field.name == "x") {
            x = field;
        } else if (field.name == "y") {
            y = field;
        } else if (field.name == "z") {
            z = field;
        } else if (field.name == "nx") {
            schema.nx = field;
        } else if (field.name == "ny") {
            schema.ny = field;
        } else if (field.name == "nz") {
            schema.nz = field;
        } else if (field.name == "t" || field.name == "timestamp" || field.name == "time") {
            schema.timestamp = field;
        }
    }
    if (!x || !y || !z) {
        throw std::runtime_error("PointCloud2 is missing one or more of x, y, z fields");
    }
    if (require_normals && (!schema.nx || !schema.ny || !schema.nz)) {
        throw std::runtime_error("PointCloud2 is missing one or more of nx, ny, nz fields");
    }
    schema.x = *x;
    schema.y = *y;
    schema.z = *z;
    validate_pointcloud_layout(msg);
    validate_field_fits_points(msg, schema.x, coordinate_field_size(schema.x));
    validate_field_fits_points(msg, schema.y, coordinate_field_size(schema.y));
    validate_field_fits_points(msg, schema.z, coordinate_field_size(schema.z));
    if (require_normals) {
        validate_field_fits_points(msg, *(schema.nx), coordinate_field_size(*(schema.nx)));
        validate_field_fits_points(msg, *(schema.ny), coordinate_field_size(*(schema.ny)));
        validate_field_fits_points(msg, *(schema.nz), coordinate_field_size(*(schema.nz)));
    }
    if (schema.timestamp) {
        validate_field_fits_points(msg, *(schema.timestamp),
                                   timestamp_field_size(*(schema.timestamp)));
    }
    return schema;
}

void extract_timestamps_from_pointcloud2_msg(const PointCloud2 &msg,
                                             const PointField &timestamp_field,
                                             std::vector<double> &timestamps) {
    auto extract_timestamps =
        [&msg, &timestamps]<typename T>(sensor_msgs::PointCloud2ConstIterator<T> &&it) {
            const size_t n_points = msg.height * msg.width;
            for (size_t i = 0; i < n_points; ++i, ++it) {
                timestamps.push_back(static_cast<double>(*it));
            }
        };

    // According to the type of the timestamp == type, return a PointCloud2ConstIterator<type>
    using sensor_msgs::PointCloud2ConstIterator;
    if (timestamp_field.datatype == PointField::UINT32) {
        extract_timestamps(PointCloud2ConstIterator<uint32_t>(msg, timestamp_field.name));
        return;
    } else if (timestamp_field.datatype == PointField::FLOAT32) {
        extract_timestamps(PointCloud2ConstIterator<float>(msg, timestamp_field.name));
        return;
    } else if (timestamp_field.datatype == PointField::FLOAT64) {
        extract_timestamps(PointCloud2ConstIterator<double>(msg, timestamp_field.name));
        return;
    } else {
        // timestamp type not supported, please open an issue :)
        throw std::runtime_error("timestamp field type not supported");
    }
}

void get_timestamps(const PointCloud2 &msg,
                    const PointCloudSchema &schema,
                    std::vector<double> &timestamps) {
    timestamps.clear();
    if (!schema.timestamp) {
        RCLCPP_WARN_ONCE(rclcpp::get_logger("slam_node"),
                         "Field 't', 'timestamp', or 'time' does not exist. "
                         "Disabling scan deskewing");
        return;
    }
    extract_timestamps_from_pointcloud2_msg(msg, schema.timestamp.value(), timestamps);
}

template <typename T>
inline T read_scalar(const uint8_t *data, const bool message_is_bigendian) {
    T value;
    std::memcpy(&value, data, sizeof(value));
    if (message_is_bigendian != (std::endian::native == std::endian::big)) {
        auto *bytes = reinterpret_cast<uint8_t *>(&value);
        std::reverse(bytes, bytes + sizeof(value));
    }
    return value;
}

inline double read_coordinate(const PointCloud2 &msg,
                              const PointField &field,
                              const uint8_t *point_data) {
    const uint8_t *field_data = point_data + field.offset;
    if (field.datatype == PointField::FLOAT32) {
        return static_cast<double>(read_scalar<float>(field_data, msg.is_bigendian));
    }
    return read_scalar<double>(field_data, msg.is_bigendian);
}

inline const uint8_t *point_data(const PointCloud2 &msg, const size_t point_index) {
    const size_t row = point_index / msg.width;
    const size_t column = point_index % msg.width;
    return msg.data.data() + row * msg.row_step + column * msg.point_step;
}

void ros_pointcloud2_to_vector_of_eigen(
    const PointCloud2 &msg,
    const PointCloudSchema &schema,
    std::vector<Eigen::Vector3d> &points,
    std::optional<Eigen::Isometry3d> transform_option = std::nullopt) {
    const size_t n_points = msg.height * msg.width;
    points.clear();
    if (!transform_option.has_value()) {
        for (size_t i = 0; i < n_points; ++i) {
            const uint8_t *data = point_data(msg, i);
            points.emplace_back(read_coordinate(msg, schema.x, data),
                                read_coordinate(msg, schema.y, data),
                                read_coordinate(msg, schema.z, data));
        }
    } else {
        const auto transform = transform_option.value();
        for (size_t i = 0; i < n_points; ++i) {
            const uint8_t *data = point_data(msg, i);
            const Eigen::Vector3d point(read_coordinate(msg, schema.x, data),
                                        read_coordinate(msg, schema.y, data),
                                        read_coordinate(msg, schema.z, data));
            points.emplace_back(transform * point);
        }
    }
}

void ros_pointcloud2_with_normal_to_vectors_of_eigen(const PointCloud2 &msg,
                                                     const PointCloudSchema &schema,
                                                     std::vector<Eigen::Vector3d> &points,
                                                     std::vector<Eigen::Vector3d> &normals) {
    points.clear();
    normals.clear();
    const size_t n_points = msg.height * msg.width;
    if (!schema.nx || !schema.ny || !schema.nz) {
        throw std::runtime_error("PointCloud2 schema does not contain normals");
    }
    for (size_t i = 0; i < n_points; ++i) {
        const uint8_t *data = point_data(msg, i);
        points.emplace_back(read_coordinate(msg, schema.x, data),
                            read_coordinate(msg, schema.y, data),
                            read_coordinate(msg, schema.z, data));
        normals.emplace_back(read_coordinate(msg, schema.nx.value(), data),
                             read_coordinate(msg, schema.ny.value(), data),
                             read_coordinate(msg, schema.nz.value(), data));
    }
}
}  // namespace pointcloud_utils
