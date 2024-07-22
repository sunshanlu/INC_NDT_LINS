#include <execution>

#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>

#include "CloudConvert.h"
#include "IEKFLio.h"

NAMESPACE_BEGIN

/// 处理IMU数据
void IEKFLio::AddIMU(const IMU::Ptr &imu) {
    if (!imu_static_init_->init_success_) {
        imu_static_init_->AddIMU(imu);
        if (imu_static_init_->init_success_) {
            iekf_->InitIEKF(imu_static_init_->acc_bias_, imu_static_init_->gyr_bias_, imu_static_init_->gravity_,
                            imu_static_init_->sigma_a_, imu_static_init_->sigma_g_);
            iekf_->last_imu_ = imu;
        }
        return;
    }

    if (first_laser_)
        return;

    if ((cloud_buffer_.size() == 1 && imu->stamp_ > end_time_buffer_.front()) ||
        (cloud_buffer_.size() > 1 && imu->stamp_ > end_time_buffer_.front() && imu->stamp_ < end_time_buffer_[1])) {
        iekf_->Predict(imu);
        Twi_query_.push_back(iekf_->GetTwi());
        stamp_query_.push_back(imu->stamp_);
        PublishNavState();
    } else if (cloud_buffer_.size() > 1 && imu->stamp_ > end_time_buffer_[1]) {
        iekf_->Predict2stamp(end_time_buffer_[1]);

        PointCloud::Ptr undistort_cloud = UndistortCloud(cloud_buffer_[1], start_time_buffer_[1]);
        undistort_cloud = VoxelCloud(undistort_cloud, 0.1);
        PointCloud::Ptr imu_cloud = pcl::make_shared<PointCloud>();
        pcl::transformPointCloud(*undistort_cloud, *imu_cloud, options_.Tli_.inverse().matrix().cast<float>());

        iekf_->ObserveIncNDT(
            [&](const SE3d &Twi, Mat18d &HTVH, Vec18d &HTVr) { inc_ndt_->GetHessian(imu_cloud, Twi, HTVH, HTVr); },
            end_time_buffer_[1], 0.01);
        PublishNavState();

        PointCloud::Ptr world_cloud = pcl::make_shared<PointCloud>();
        if (viewer_) {
            pcl::transformPointCloud(*imu_cloud, *world_cloud, iekf_->GetTwi().matrix().cast<float>());
            viewer_->SetPoseAndCloud(iekf_->GetTwi(), world_cloud);
        }
        ++frame_cnt_;
        if (IsKeyframe()) {
            frame_cnt_ = 0;
            last_key_pose_ = iekf_->GetTwi();
            if (!viewer_)
                pcl::transformPointCloud(*imu_cloud, *world_cloud, iekf_->GetTwi().matrix().cast<float>());

            inc_ndt_->AddCloud(world_cloud);
        }

        cloud_buffer_.pop_front();
        start_time_buffer_.pop_front();
        end_time_buffer_.pop_front();
        AddIMU(imu);
    }
}

/**
 * @brief 向紧耦合系统中添加点云数据信息
 *
 * @param cloud 输入的点云数据
 * @param laser_start_stamp 输入的点云数据对应的开始时间
 */
void IEKFLio::AddCloud(const FullPointCloud::Ptr &cloud, double laser_start_stamp) {
    if (!imu_static_init_->init_success_)
        return;

    if (first_laser_) {
        PointCloud::Ptr pcl_cloud = CloudConvert::Full2PointCloud(cloud);
        pcl_cloud = VoxelCloud(pcl_cloud, 0.1);
        PointCloud::Ptr point_cloud = pcl::make_shared<PointCloud>();
        pcl::transformPointCloud(*pcl_cloud, *point_cloud, options_.Tli_.inverse().matrix().cast<float>());
        inc_ndt_->AddCloud(point_cloud);

        last_key_pose_ = SE3d();
        iekf_->SetTwi(last_key_pose_, laser_start_stamp + cloud->back().time);
        iekf_->last_stamp_ = laser_start_stamp + cloud->back().time;
        first_laser_ = false;
        PublishNavState();
        if (viewer_)
            viewer_->SetPoseAndCloud(last_key_pose_, point_cloud);
    }

    cloud_buffer_.push_back(cloud);
    start_time_buffer_.push_back(laser_start_stamp);
    end_time_buffer_.push_back(laser_start_stamp + cloud->back().time);
}

// todo 对点云去畸变和位姿差值进行重构，减少重复代码
/**
 * @brief 使用位姿差值进行点云的运动矫正
 *
 * @return PointCloud::Ptr 输出的矫正后的普通点云信息，用于后续的配准工作
 */
PointCloud::Ptr IEKFLio::UndistortCloud(const FullPointCloud::Ptr &full_cloud, const double &laser_start_time) {
    SE3d Tlw_ts = options_.Tli_ * iekf_->GetTwi().inverse();
    std::vector<int> indices(full_cloud->size());
    std::for_each(indices.begin(), indices.end(), [idx = 0](int &id) mutable { id = idx++; });

    std::vector<PointT> points_out(full_cloud->size());
    std::vector<bool> points_valid(full_cloud->size(), true);
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
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
bool IEKFLio::DifferencePose(const double &query_stamp, SE3d &Twl_t) {
    const double &earliest_time = stamp_query_.front();
    const double &latest_time = stamp_query_.back();

    if (query_stamp <= earliest_time) {
        /// 位姿差值超出时间范围50ms
        if (earliest_time - query_stamp > 0.05)
            return false;
        Twl_t = Twi_query_.front() * options_.Tli_.inverse();
        return true;
    }

    if (query_stamp >= latest_time) {
        if (query_stamp - latest_time > 0.05)
            return false;
        Twl_t = Twi_query_.back() * options_.Tli_.inverse();
        return true;
    }

    for (int i = 0; i < stamp_query_.size() - 1; ++i) {
        const double &stamp_i = stamp_query_[i];
        const double &stamp_j = stamp_query_[i + 1];
        if (stamp_i < query_stamp && stamp_j >= query_stamp) {
            double dt = stamp_j - stamp_i;
            if (dt < 1e-4) {
                Twl_t = Twi_query_[i] * options_.Tli_.inverse();
                break;
            }
            double s = (query_stamp - stamp_i) / dt;
            Vec3d t_interp = s * Twi_query_[i].translation() + (1 - s) * Twi_query_[i + 1].translation();
            Eigen::Quaterniond R_interp = Twi_query_[i].unit_quaternion().slerp(s, Twi_query_[i + 1].unit_quaternion());
            Twl_t = SE3d(R_interp, t_interp) * options_.Tli_.inverse();
            break;
        }
    }
    return true;
}

/**
 * @brief 判断是否是关键帧，是否需要向inc-ndt中添加点云信息
 *
 * @return true     是关键帧
 * @return false    不是关键帧
 */
bool IEKFLio::IsKeyframe() {
    // if (frame_cnt_ > 10)
    //     return true;

    SE3d curr_pose = iekf_->GetTwi();
    SE3d Tlc = last_key_pose_.inverse() * curr_pose;
    if (Tlc.translation().lpNorm<2>() > options_.delta_position_)
        return true;
    else if (Tlc.so3().log().lpNorm<2>() > options_.delta_angle_)
        return true;
    return false;
}

/// 向外部发送导航状态
void IEKFLio::PublishNavState() {
    // todo 进行导航状态输出
    Vec3d position = iekf_->x_normal_.Twi_.translation();
    Vec4d quat = iekf_->x_normal_.Twi_.unit_quaternion().coeffs();
    Vec3d velocity = iekf_->x_normal_.v_;
    Vec3d acc = iekf_->last_imu_->acc_;
    if (iekf_->options_.remove_gravity_)
        acc = iekf_->x_normal_.Twi_.so3().inverse() * acc - iekf_->options_.a_bias_;
    else
        acc = iekf_->x_normal_.Twi_.so3().inverse() * (acc - iekf_->x_normal_.g_) - iekf_->options_.a_bias_;

    // clang-format off
    RCLCPP_INFO_STREAM(rclcpp::get_logger("inc_ndt_ins"), 
        "stamp: " << iekf_->x_normal_.stamp_ <<
        " pose_t: " << position.transpose() << 
        " pose_r: " << quat.transpose() <<
        " velocity: " << velocity.transpose() <<
        " acc: " << acc.transpose();
    );
    // clang-format on
}

NAMESPACE_END
