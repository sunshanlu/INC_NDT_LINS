#pragma once

#include "Common.hpp"
#include "ESKF.h"
#include "IMUPreintegration.h"
#include "IncNdt.h"

NAMESPACE_BEGIN

class PreinteLio {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Options {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool remove_gravity_;     ///< 是否移除重力
        SE3d Tli_;                ///< 雷达到imu的变换
        double delta_position_;   ///< 关键帧t差
        double delta_angle_;      ///< 关键帧r差
        double ndt_position_var_; ///< 雷达位置标准差
        double ndt_angle_var_;    ///< 雷达角度标准差
    };

    PreinteLio(Options options)
        : imu_static_init_(nullptr)
        , inc_ndt_(nullptr)
        , preint_(nullptr)
        , first_laser_(true)
        , prior_info_(Mat15d::Identity() * 1e8)
        , ndt_info_(Mat6d::Zero())
        , options_(std::move(options)) {
        double ep2 = options_.ndt_position_var_ * options_.ndt_position_var_;
        double ea2 = options_.ndt_angle_var_ * options.ndt_angle_var_;
        ndt_info_.diagonal() << ep2, ep2, ep2, ea2, ea2, ea2;
    }

    /// 添加imu数据
    void AddIMU(const IMU::Ptr &imu);

    /// 添加laser数据
    void AddCloud(const FullPointCloud::Ptr &full_cloud, double laser_start_time);

    /// 设置增量ndt配准器
    void SetIncNdt(IncNdt::Ptr inc_ndt) { inc_ndt_ = std::move(inc_ndt); }

    /// 设置IMU初始化器
    void SetImuInit(IMUStaticInit::Ptr imu_static_init) { imu_static_init_ = std::move(imu_static_init); }

    /// 设置IMU预积分器
    void SetImuPreint(IMUPreintegration::Ptr preint) { preint_ = std::move(preint); }

private:
    /// 点云去畸变
    PointCloud::Ptr UndistortCloud(const FullPointCloud::Ptr &full_cloud, const double &laser_start_time);

    /// 位姿差值
    bool DifferencePose(const double &query_stamp, SE3d &Twl_t);

    /// 优化
    void Optimize(const SE3d &ndt_pose);

    /// 是否为关键帧
    bool IsKeyframe();

    SE3d last_key_Twi_;    ///< 关键帧位姿
    NavState state_timei_; ///< i时刻的系统状态
    NavState state_timej_; ///< j时刻的系统状态

    IMUStaticInit::Ptr imu_static_init_; ///< IMU的静态初始化器
    IncNdt::Ptr inc_ndt_;                ///< 增量ndt里程计
    IMUPreintegration::Ptr preint_;      ///< imu的预积分器
    bool first_laser_;                   ///< 预积分器初始化标志
    Mat15d prior_info_;                  ///< 先验信息矩阵
    Mat6d ndt_info_;                     ///< ndt观测SE3的信息矩阵

    std::deque<FullPointCloud::Ptr> cloud_buffer_; ///< 雷达缓冲区
    std::deque<double> start_time_buffer_;         ///< 雷达开始时间
    std::deque<double> end_time_buffer_;           ///< 雷达结束时间

    std::vector<SE3d> Twi_query_;     ///< imu位姿查询
    std::vector<double> stamp_query_; ///< 时间查询

    Options options_; ///< 预积分LIO配置选项
};

NAMESPACE_END