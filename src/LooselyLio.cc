#include <execution>

#include "CloudConvert.h"
#include "LooselyLio.h"

NAMESPACE_BEGIN

/**
 * @brief 添加IMU数据信息
 *
 * @param imu 输入的imu数据
 */
void LooselyLio::AddIMU(const IMU::Ptr &imu) {
    if (!eskf_->ImuInitSuccess()) {
        eskf_->AddIMU(imu);
        return;
    }

    if (!eskf_->ImuInitSuccess() || first_se3_)
        return;

    if ((imu->stamp_ > end_time_buffer_.front() && laser_buffer_.size() < 2) ||
        (laser_buffer_.size() >= 2 && imu->stamp_ < end_time_buffer_[1])) {
        eskf_->AddIMU(imu);
        stamp_query_.push_back(imu->stamp_);
        Twi_query_.push_back(eskf_->GetTwi());
        PublishNavState();
    } else if (laser_buffer_.size() >= 2 && imu->stamp_ >= end_time_buffer_[1]) {
        undistort_cloud_ = UndistortCloud(laser_buffer_[1], start_time_buffer_[1]);
        
        Align();

        laser_buffer_.pop_front();
        start_time_buffer_.pop_front();
        end_time_buffer_.pop_front();
        AddIMU(imu);
        PublishNavState();
    }
}

/**
 * @brief 添加点云数据
 *
 * @param cloud 输入的点云数据
 * @param stamp 输入的时间戳数据
 */
void LooselyLio::AddCloud(const FullPointCloud::Ptr &cloud, const double &laser_start_time) {
    if (!eskf_->ImuInitSuccess())
        return;

    laser_buffer_.push_back(cloud);
    start_time_buffer_.push_back(laser_start_time);
    end_time_buffer_.push_back(laser_start_time + cloud->points.back().time);
    if (first_se3_) {
        SE3d Twl;
        PointCloud::Ptr first_cloud = CloudConvert::Full2PointCloud(cloud);
        ndt_lo_->AddCloud(VoxelCloud(first_cloud, 0.5), Twl);
        first_se3_ = false;
        eskf_->AddObserveSE3(Tli_, end_time_buffer_.front());
        PublishNavState();
        return;
    }
}

/**
 * @brief 使用位姿差值进行点云的运动矫正
 *
 * @return PointCloud::Ptr 输出的矫正后的普通点云信息，用于后续的配准工作
 */
PointCloud::Ptr LooselyLio::UndistortCloud(const FullPointCloud::Ptr &full_cloud, const double &laser_start_time) {
    SE3d Tlw_ts = Tli_ * eskf_->x_normal_.Twi_.inverse();
    std::vector<int> indices(full_cloud->size());
    std::for_each(indices.begin(), indices.end(), [idx = 0](int &id) mutable { id = idx++; });

    std::vector<PointT> points_out(full_cloud->size());
    std::vector<bool> points_valid(full_cloud->size(), true);
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
        // std::for_each(indices.begin(), indices.end(), [&](const int &idx) {
        SE3d Twl_t;
        const auto &pt_in = full_cloud->points[idx];
        if (!DifferencePose(laser_start_time + pt_in.time, Twl_t)) {
            points_valid[idx] = false;
            return;
        }
        Vec3d pt_ts = Tlw_ts * Twl_t * to_vec3d(pt_in);
        PointT pt_out;
        pt_out.x = pt_ts[0];
        pt_out.y = pt_ts[1];
        pt_out.z = pt_ts[2];
        points_out[idx] = pt_out;
    });

    PointCloud::Ptr pcl_out = pcl::make_shared<PointCloud>();
    std::for_each(indices.begin(), indices.end(), [&](const int &idx) {
        if (points_valid[idx]) {
            pcl_out->push_back(points_out[idx]);
        }
    });

    /// 重新组织位姿差值查询数据库
    std::vector<double> stamp_query;
    std::vector<SE3d> Twi_query;

    std::swap(stamp_query, stamp_query_);
    std::swap(Twi_query, Twi_query_);

    return pcl_out;
}

/// 对位姿进行差值
bool LooselyLio::DifferencePose(const double &query_stamp, SE3d &Twl_t) {
    const double &earliest_time = stamp_query_.front();
    const double &latest_time = stamp_query_.back();

    if (query_stamp <= earliest_time) {
        /// 位姿差值超出时间范围50ms
        if (earliest_time - query_stamp > 0.05)
            return false;
        Twl_t = Twi_query_.front() * Tli_.inverse();
        return true;
    }

    if (query_stamp >= latest_time) {
        if (query_stamp - latest_time > 0.05)
            return false;
        Twl_t = Twi_query_.back() * Tli_.inverse();
        return true;
    }

    for (int i = 0; i < stamp_query_.size() - 1; ++i) {
        const double &stamp_i = stamp_query_[i];
        const double &stamp_j = stamp_query_[i + 1];
        if (stamp_i < query_stamp && stamp_j >= query_stamp) {
            double dt = stamp_j - stamp_i;
            if (dt < 1e-4) {
                Twl_t = Twi_query_[i] * Tli_.inverse();
                break;
            }
            double s = (query_stamp - stamp_i) / dt;
            Vec3d t_interp = s * Twi_query_[i].translation() + (1 - s) * Twi_query_[i + 1].translation();
            Eigen::Quaterniond R_interp = Twi_query_[i].unit_quaternion().slerp(s, Twi_query_[i + 1].unit_quaternion());
            Twl_t = SE3d(R_interp, t_interp) * Tli_.inverse();
            break;
        }
    }
    return true;
}

/// 向外部发送导航状态
void LooselyLio::PublishNavState() {
    // todo 进行导航状态输出
    // clang-format off
    RCLCPP_INFO_STREAM(rclcpp::get_logger("inc_ndt_ins"), 
        "pose_t: " << eskf_->x_normal_.Twi_.translation().transpose() << 
        " pose_r: " << eskf_->x_normal_.Twi_.unit_quaternion().coeffs().transpose() <<
        " velocity: " << eskf_->x_normal_.v_.transpose() <<
        " acc: " << eskf_->GetAccerate().transpose()
    );
    // clang-format on
}

void LooselyLio::Align() {
    PointCloud::Ptr pcl_out = VoxelCloud(undistort_cloud_, 0.1);
    eskf_->Predice2stamp(end_time_buffer_[1]);
    SE3d pose_predict = eskf_->GetTwi();
    ndt_lo_->AddCloud(pcl_out, pose_predict);
    eskf_->AddObserveSE3(pose_predict * Tli_, end_time_buffer_[1]);
}

NAMESPACE_END
