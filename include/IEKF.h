#pragma once

#include "Common.hpp"
#include "ESKF.h"

NAMESPACE_BEGIN

class IEKFLio;

class IEKF {
    friend class IEKFLio;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<IEKF> Ptr;
    typedef std::function<void(const SE3d &, Mat18d &, Vec18d &)> ObserveMatrixFunc;

    struct Options {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int max_iterations_;  ///< 最大迭代次数
        double quit_eps_;     ///< 退出迭代条件
        double a_bias_var_;   ///< 加速度计偏置标准差
        double g_bias_var_;   ///< 陀螺仪偏置标准差
        bool remove_gravity_; ///< imu读数是否移除重力
        Vec3d a_bias_;        ///< 加速度偏置
        Vec3d g_bias_;        ///< 陀螺仪偏置
        double acc_var_;      ///< 加速度计测量噪声
        double gyr_var_;      ///< 陀螺仪测量噪声
    };

    /// IEKF默认构造
    IEKF(Options options)
        : last_imu_(nullptr)
        , P_(Mat18d::Zero())
        , F_(Mat18d::Identity())
        , Q_(Mat18d::Zero())
        , options_(std::move(options))
        , delta_t_(0)
        , last_stamp_(0) {}

    /// 增量ndt的迭代观测方程
    void ObserveIncNDT(const ObserveMatrixFunc &matrix_func, const double &stamp, double info_ratio = 0.01);

    /// IEKF的预测阶段
    void Predict(const IMU::Ptr &imu);

    /// 使用最后的imu预测到雷达结束时间
    void Predict2stamp(const double &laser_end_time) {
        delta_t_ = laser_end_time - last_stamp_;
        last_stamp_ = laser_end_time;
        UpdateNormalState();
        ComputeFJacobian();
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    /// 初始化IEKF系统
    void InitIEKF(const Vec3d &ba, const Vec3d &bg, const Vec3d &gravity, double acc_var, double gyr_var) {
        options_.a_bias_ = ba;
        options_.g_bias_ = bg;
        options_.acc_var_ = acc_var;
        options_.gyr_var_ = gyr_var;
        x_normal_.g_ = gravity;
        BuildNoise();
    }

    /// 获取当前状态的位姿
    SE3d GetTwi() { return x_normal_.Twi_; }

    /// 设置当前状态的位姿
    void SetTwi(const SE3d &Twi, const double stamp) {
        x_normal_.Twi_ = Twi;
        x_normal_.stamp_ = stamp;
    }

private:
    /// 更新名义状态向量
    void UpdateNormalState();

    /// 计算预测模型线性化后的状态转移矩阵
    void ComputeFJacobian();

    /// 构建Q噪声矩阵
    void BuildNoise();

    IMU::Ptr last_imu_; ///< 最新的imu数据
    Mat18d P_;          ///< 误差状态的协方差矩阵
    Mat18d F_;          ///< 误差状态线性化雅可比
    Mat18d Q_;          ///< IMU的预测模型协方差矩阵
    NavState x_normal_; ///< 名义状态向量
    Options options_;   ///< 迭代卡尔曼配置参数
    double delta_t_;    ///< 距离上一次预测或观测的时间间隔
    double last_stamp_; ///< 上一次预测或观测时间戳
};

NAMESPACE_END