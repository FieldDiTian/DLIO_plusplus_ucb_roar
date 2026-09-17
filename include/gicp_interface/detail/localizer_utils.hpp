#pragma once

// Internal utilities shared by the node implementation translation units.
//
// Keep implementations in src/localizer_utils.cpp.  The predecessor header
// contained roughly 900 lines of function bodies and was included by every node
// source file, which duplicated private helpers across translation units and
// obscured which code belonged to the node versus point-cloud utilities.

#include "gicp_interface/gicp_interface_node.hpp"
#include "gicp_interface/urdf_transforms.hpp"

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace gicp_localizer::detail {

bool matrixFinite(const Eigen::Matrix4f& pose);

geometry_msgs::msg::Pose poseMsgFromMatrix(const Eigen::Matrix4f& pose);

double rotationDistanceDeg(const Eigen::Matrix4f& a,
                           const Eigen::Matrix4f& b);

double deltaTranslationNorm(const Eigen::Matrix4f& from,
                            const Eigen::Matrix4f& to);

std::string poseSummary(const Eigen::Matrix4f& pose);

std::string scalarSummary(double value, int precision = 3);

bool findXYZOffsets(const sensor_msgs::msg::PointCloud2& msg, int& x_off,
                    int& y_off, int& z_off);

bool auxSchemaMatchesPrimary(const sensor_msgs::msg::PointCloud2& aux,
                             const sensor_msgs::msg::PointCloud2& primary,
                             std::string& reason);

void transformCloudData(uint8_t* data, size_t num_points, uint32_t point_step,
                        int x_off, int y_off, int z_off,
                        const Eigen::Matrix4f& transform);

const char* pointFieldDatatypeName(uint8_t datatype);

void logTimestampDiagnostic(const sensor_msgs::msg::PointCloud2& cloud,
                            int time_offset, uint8_t time_datatype,
                            int time_count, const char* sensor_name);

bool findTimeField(const sensor_msgs::msg::PointCloud2& msg, int& time_offset,
                   uint8_t& time_datatype, int& time_count);

uint64_t luminarPointTimestampNs(const PointType& point);

void clearPointTimeUnion(PointType& point);

bool luminarUsesRawEpochCarrier(uint8_t time_datatype, int time_count,
                                bool float64_time_is_epoch_ns);

bool luminarUsesRelativeFloat64Carrier(uint8_t time_datatype, int time_count,
                                       bool float64_time_is_epoch_ns);

bool luminarCloudUsesRelativeFloat64(
    const sensor_msgs::msg::PointCloud2& cloud,
    bool float64_time_is_epoch_ns);

bool luminarFloat64TimeContractMatches(
    const sensor_msgs::msg::PointCloud2& cloud,
    bool float64_time_is_epoch_ns, std::string& reason);

bool luminarRawTimestampNsFromBytes(
    const uint8_t* time_data, uint8_t time_datatype, int time_count,
    size_t bytes_available, bool float64_time_is_epoch_ns,
    uint64_t& timestamp_ns);

LuminarTimestampRangeNs luminarTimestampRangeFromCloud(
    const sensor_msgs::msg::PointCloud2& cloud,
    bool float64_time_is_epoch_ns);

void copyPointTimeFromCloud(const uint8_t* source, int time_offset,
                            uint8_t time_datatype, int time_count,
                            uint32_t point_step, SensorType sensor,
                            bool float64_time_is_epoch_ns,
                            PointType& destination);

void logLuminarTimestampStats(size_t num_points,
                              const pcl::PointCloud<PointType>& cloud,
                              size_t unique_ros_times,
                              bool raw_epoch_ns);

void shiftCloudTimestamps(uint8_t* data, size_t num_points,
                          uint32_t point_step, int time_offset,
                          uint8_t time_datatype, int time_count,
                          double offset_seconds, bool raw_epoch_carrier,
                          double absolute_clock_shift_seconds = 0.0);

}  // namespace gicp_localizer::detail
