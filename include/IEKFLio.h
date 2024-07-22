#pragma once

#include "Common.hpp"
#include "IEKF.h"
#include "IncNdt.h"
#include "Viewer.h"

NAMESPACE_BEGIN

class IEKFLio {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<IEKFLio> Ptr;

    struct Options {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double delta_position_; ///< 关键帧t差
        double delta_angle_;    ///< 关键帧r差
        SE3d Tli_;              ///< 雷达到IMU的变换
    };

    IEKFLio(Options options)
        : iekf_(nullptr)
        , inc_ndt_(nullptr)
        , imu_static_init_(nullptr)
        , first_laser_(true)
        , frame_cnt_(0)
        , options_(std::move(options)) {}

    /// 处理IMU数据
    void AddIMU(const IMU::Ptr &imu);

    /// 处理点云数据
    void AddCloud(const FullPointCloud::Ptr &cloud, double laser_start_stamp);

    /// 设置IEKF系统
    void SetIEKF(IEKF::Ptr iekf) { iekf_ = std::move(iekf); }

    /// 设置INCNDT系统
    void SetINCNDT(IncNdt::Ptr inc_ndt) { inc_ndt_ = std::move(inc_ndt); }

    /// 设置IMU初始化器
    void SetIMUInit(IMUStaticInit::Ptr imu_static_init) { imu_static_init_ = std::move(imu_static_init); }

    /// 设置可视化器
    void SetViewer(Viewer::Ptr viewer) { viewer_ = std::move(viewer); }

private:
    /// 点云去畸变
    PointCloud::Ptr UndistortCloud(const FullPointCloud::Ptr &full_cloud, const double &laser_start_time);

    /// 位姿差值
    bool DifferencePose(const double &query_stamp, SE3d &Twl_t);

    /// 向外部发送导航状态
    void PublishNavState();

    /// 是否为关键帧
    bool IsKeyframe();

    IEKF::Ptr iekf_;                     ///< 迭代卡尔曼滤波器
    IncNdt::Ptr inc_ndt_;                ///< 增量NDT配准器
    IMUStaticInit::Ptr imu_static_init_; ///< IMU静态初始化器
    Viewer::Ptr viewer_;                 ///< 可视化器（pcl）

    std::deque<FullPointCloud::Ptr> cloud_buffer_; ///< 雷达数据缓冲
    std::deque<double> start_time_buffer_;         ///< 雷达开始时间缓冲
    std::deque<double> end_time_buffer_;           ///< 雷达结束时间缓冲

    std::vector<SE3d> Twi_query_;     ///< 缓冲Twi的查询
    std::vector<double> stamp_query_; ///< 缓存Twi的时间戳
    SE3d last_key_pose_;              ///< 上一关键帧位姿

    bool first_laser_;  ///< 是否是第一个激光数据
    int frame_cnt_ = 0; ///<  雷达帧记数，判断是否为关键帧
    Options options_;   ///< LIO配置选项
};

NAMESPACE_END