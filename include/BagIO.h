#pragma once

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include "CloudConvert.h"
#include "Common.hpp"
#include "ESKF.h"

NAMESPACE_BEGIN

class BagIO {
    typedef std::shared_ptr<rosbag2_storage::SerializedBagMessage> MetaData;
    typedef std::function<void(MetaData)> MessageCallback;
    typedef std::function<void(FullPointCloud::Ptr, double)> PointCloudCallback;
    typedef std::function<void(IMU::Ptr)> IMUCallback;
    typedef std::unordered_map<std::string, MessageCallback> CallbackMap;
    typedef std::unordered_map<std::string, CloudConvert::Ptr> ConvertMap;
    typedef rosbag2_cpp::readers::SequentialReader BagReader;
    typedef rclcpp::Serialization<PointCloudMsg> LaserSerialization;
    typedef rclcpp::Serialization<LivoxCloud> LivoxSerialization;
    typedef rclcpp::Serialization<ImuMsg> ImuSerialization;

public:
    BagIO(const std::string &bag_path);

    /// 设置点云回调函数
    BagIO &SetPointCloudCallback(const PointCloudCallback &callback, const std::string &topic_name,
                                 const CloudConvert::Options &options);

    /// 设置IMU回调函数
    BagIO &SetIMUCallback(const IMUCallback &callback, const std::string &topic_name);

    void Go();

private:
    ConvertMap convert_map_;                ///< 点云转换器,转换为FullPointCloud
    CallbackMap callbacks_;                 ///< 回调函数哈希表
    std::unique_ptr<BagReader> bag_reader_; ///< rosbag序列化读取器
    LaserSerialization laser_serializer_;   ///< 激光序列化器
    LivoxSerialization livox_serializer_;   ///< livox固态序列化器
    ImuSerialization imu_serializer_;       ///< imu序列化器
};

NAMESPACE_END