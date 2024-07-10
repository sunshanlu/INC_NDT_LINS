#pragma once

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include "Common.hpp"

NAMESPACE_BEGIN

class BagIO {
    typedef std::shared_ptr<rosbag2_storage::SerializedBagMessage> MetaData;
    typedef std::function<void(MetaData)> MessageCallback;
    typedef std::function<void(PointCloud::Ptr)> PointCloudCallback;
    typedef std::unordered_map<std::string, MessageCallback> CallbackMap;
    typedef rosbag2_cpp::readers::SequentialReader BagReader;
    typedef rclcpp::Serialization<PointCloudMsg> LaserSerialization;

public:
    BagIO(const std::string &bag_path);

    /// 读取bag中的点云数据
    PointCloud::Ptr ReadPointCloud();

    /// 设置点云回调函数
    BagIO &SetPointCloudCallback(const PointCloudCallback &callback, const std::string &topic_name);

    void Go();

private:
    std::string point_cloud_topic_;         ///< 点云话题名
    CallbackMap callbacks_;                 ///< 回调函数哈希表
    std::unique_ptr<BagReader> bag_reader_; ///< rosbag序列化读取器
    LaserSerialization laser_serializer_;   ///< 激光序列化器
};

NAMESPACE_END