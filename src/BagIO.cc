#include <pcl_conversions/pcl_conversions.h>

#include "BagIO.h"

NAMESPACE_BEGIN

/// BagIO的构造
BagIO::BagIO(const std::string &bag_path) {
    bag_reader_ = std::make_unique<BagReader>();
    bag_reader_->open({bag_path, "sqlite3"}, {"cdr", "cdr"});
}

/**
 * @brief 读取bag数据包中的点云数据
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

NAMESPACE_END