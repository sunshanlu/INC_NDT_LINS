#include <pcl_conversions/pcl_conversions.h>

#include "BagIO.h"

NAMESPACE_BEGIN

/// BagIO的构造
BagIO::BagIO(const std::string &bag_path) {
    bag_reader_ = std::make_unique<BagReader>();
    bag_reader_->open({bag_path, "sqlite3"}, {"cdr", "cdr"});
}

/**
 * @brief 设置点云回调函数
 *
 * @param callback      输入的回调函数
 * @param topic_name    输入的topic名称
 * @return BagIO& 输出的BagIO对象，链式编程
 */
BagIO &BagIO::SetPointCloudCallback(const PointCloudCallback &callback, const std::string &topic_name,
                                    const CloudConvert::Options &options) {

    convert_map_[topic_name] = std::make_shared<CloudConvert>(options);

    MessageCallback laser_callback = [&](MetaData meta_data) {
        FullPointCloud::Ptr pcl_cloud;
        rclcpp::SerializedMessage sm(*meta_data->serialized_data);
        auto point_cloud = std::make_shared<PointCloudMsg>();
        auto livox_cloud = std::make_shared<LivoxCloud>();
        switch (options.cloud_type_) {
        case CloudConvert::CloudType::VELO:
        case CloudConvert::CloudType::OUST:
            laser_serializer_.deserialize_message(&sm, point_cloud.get());
            convert_map_[topic_name]->Process(point_cloud, pcl_cloud);
            break;
        case CloudConvert::CloudType::AVIA:
            livox_serializer_.deserialize_message(&sm, point_cloud.get());
            convert_map_[topic_name]->Process(point_cloud, pcl_cloud);
            break;

        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "程序不支持该点云类型");
            return;
        }
        double stamp = point_cloud->header.stamp.sec + point_cloud->header.stamp.nanosec * 1e-9;
        callback(pcl_cloud, stamp);
    };

    callbacks_.insert({topic_name, laser_callback});
    return *this;
}

BagIO &BagIO::SetIMUCallback(const IMUCallback &callback, const std::string &topic_name) {
    MessageCallback imu_callback = [&](MetaData meta_data) {
        rclcpp::SerializedMessage sm(*meta_data->serialized_data);
        auto imu_msg = std::make_shared<ImuMsg>();
        imu_serializer_.deserialize_message(&sm, imu_msg.get());

        const auto &acc = imu_msg->linear_acceleration;
        const auto &gyr = imu_msg->angular_velocity;

        IMU::Ptr imu = std::make_shared<IMU>();
        imu->acc_ = Vec3d(acc.x, acc.y, acc.z);
        imu->gyr_ = Vec3d(gyr.x, gyr.y, gyr.z);
        imu->stamp_ = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;

        callback(imu);
    };

    callbacks_.insert({topic_name, imu_callback});
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