#include <pcl_conversions/pcl_conversions.h>

#include "BagIO.h"

NAMESPACE_BEGIN

/// BagIO的构造
BagIO::BagIO(const std::string &bag_path) {
    bag_reader_ = std::make_unique<BagReader>();
    bag_reader_->open({bag_path, "sqlite3"}, {"cdr", "cdr"});
}

/**
 * @brief 读取bag数据包中的点云数据，方便调试
 *
 * @return PointCloud::Ptr pcl的点云信息
 */
PointCloud::Ptr BagIO::ReadPointCloud() {
    while (bag_reader_->has_next()) {
        auto meta_data = bag_reader_->read_next();
        if (!meta_data || meta_data->topic_name != point_cloud_topic_)
            continue;
        auto point_cloud = std::make_shared<PointCloudMsg>();
        rclcpp::SerializedMessage sm(*meta_data->serialized_data);
        laser_serializer_.deserialize_message(&sm, point_cloud.get());
        PointCloud::Ptr pcl_cloud = pcl::make_shared<PointCloud>();
        pcl::fromROSMsg(*point_cloud, *pcl_cloud);
        return pcl_cloud;
    }
    return nullptr;
}

/**
 * @brief 设置点云回调函数
 *
 * @param callback      输入的回调函数
 * @param topic_name    输入的topic名称
 * @return BagIO& 输出的BagIO对象，链式编程
 */
BagIO &BagIO::SetPointCloudCallback(const PointCloudCallback &callback, const std::string &topic_name) {

    MessageCallback laser_callback = [&](MetaData meta_data) {
        auto point_cloud = std::make_shared<PointCloudMsg>();
        rclcpp::SerializedMessage sm(*meta_data->serialized_data);
        laser_serializer_.deserialize_message(&sm, point_cloud.get());
        PointCloud::Ptr pcl_cloud = pcl::make_shared<PointCloud>();
        pcl::fromROSMsg(*point_cloud, *pcl_cloud);
        callback(pcl_cloud);
    };

    callbacks_.insert({topic_name, laser_callback});
    return *this;
}

/**
 * @brief BagIO自动化函数
 *
 */
void BagIO::Go() {
    while (bag_reader_->has_next()) {
        MetaData meta_data = bag_reader_->read_next();
        if (!meta_data)
            continue;
        auto iter = callbacks_.find(meta_data->topic_name);
        if (iter == callbacks_.end())
            continue;

        iter->second(meta_data);
    }
}

NAMESPACE_END