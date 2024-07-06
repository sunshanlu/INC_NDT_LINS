#pragma once

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include "Common.hpp"

NAMESPACE_BEGIN

class BagIO {
    typedef std::function<bool(PointCloud::Ptr)> PointCloudCallback;
    typedef rosbag2_cpp::readers::SequentialReader BagReader;
    typedef rclcpp::Serialization<PointCloudMsg> LaserSerialization;

public:
    BagIO(const std::string &bag_path);

    /// 读取bag中的点云数据
    PointCloud::Ptr ReadPointCloud();

    /// 设置点云回调函数
    BagIO &SetPointCloudCallback(const PointCloudCallback &callback, const std::string &topic_name) {
        point_cloud_callback_ = callback;
        point_cloud_topic_ = topic_name;
        return *this;
    }

private:
    std::string point_cloud_topic_;           ///< 点云话题名
    PointCloudCallback point_cloud_callback_; ///< 点云回调函数
    std::unique_ptr<BagReader> bag_reader_;   ///< rosbag序列化读取器
    LaserSerialization laser_serializer_;     ///< 激光序列化器
};

NAMESPACE_END