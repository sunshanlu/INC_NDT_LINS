#pragma once

#include "Common.hpp"
#include "ESKF.h"

NAMESPACE_BEGIN

class IntegEdge;

class IMUPreintegration {
    friend class IntegEdge;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<IMUPreintegration> Ptr;

    struct Options {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool remove_gravity_; ///< imu读数是否去除重力
        Vec3d acc_bias_;      ///< 加速度计偏置
        Vec3d gyr_bias_;      ///< 陀螺仪偏置
        Vec3d gravity_;       ///< 重力
        double acc_var_;      ///< 加速度计标准差
        double gyr_var_;      ///< 陀螺仪标准差
    };

    IMUPreintegration(Options options)
        : integrate_time_(0)
        , dR_(SO3d())
        , dv_(Vec3d::Zero())
        , dp_(Vec3d::Zero())
        , cov_(Mat9d::Zero())
        , bias_cov_(Mat6d::Zero())
        , dr_dbg_(Mat3d::Zero())
        , dv_dba_(Mat3d::Zero())
        , dv_dbg_(Mat3d::Zero())
        , dp_dba_(Mat3d::Zero())
        , dp_dbg_(Mat3d::Zero())
        , options_(std::move(options)) {
        acc_bias_ = options_.acc_bias_;
        gyr_bias_ = options_.gyr_bias_;
        const double &da2 = options_.acc_var_ * options_.acc_var_;
        const double &dg2 = options_.gyr_var_ * options_.gyr_var_;
        bias_cov_.diagonal() << dg2, dg2, dg2, da2, da2, da2;
    }

    /// 积分api
    void Integrate(const IMU::Ptr &imu, double stamp);

    void Reset(const Vec3d &bg, const Vec3d &ba) {
        dR_ = SO3d();
        dv_ = Vec3d::Zero();
        dp_ = Vec3d::Zero();
        cov_ = Mat9d::Zero();
        dr_dbg_.setZero();
        dv_dba_.setZero();
        dv_dbg_.setZero();
        dp_dba_.setZero();
        dp_dbg_.setZero();
        acc_bias_ = ba;
        gyr_bias_ = bg;
        integrate_time_ = 0;
    }

    /// 输入改变的bg，获取dR的修正输出
    SO3d GetDeltaR(const Vec3d &bg) { return dR_ * SO3d::exp(dr_dbg_ * (bg - gyr_bias_)); }

    /// 输入改变的ba和bg，输出修正后的dv
    Vec3d GetDeltaV(const Vec3d &bg, const Vec3d &ba) {
        Vec3d delta_ba = ba - acc_bias_;
        Vec3d delta_bg = bg - gyr_bias_;
        return dv_ + dv_dbg_ * delta_bg + dv_dba_ * delta_ba;
    }

    /// 输入改变后的ba和bg，输出修正后的dp
    Vec3d GetDeltaP(const Vec3d &bg, const Vec3d &ba) {
        Vec3d delta_ba = ba - acc_bias_;
        Vec3d delta_bg = bg - gyr_bias_;
        return dp_ + dp_dbg_ * delta_bg + dp_dba_ * delta_ba;
    }

    /// 获取预积分模型的信息矩阵
    Mat9d GetInformation();

    /// 输入i时刻状态，输出积分时刻的状态
    NavState Predict(const NavState &state_i);

    /// 当雷达初始化后，调用
    void SetLaserStamp(double laser_end_time) { last_stamp_ = std::move(laser_end_time); }

private:
    /// 更新积分状态
    void UpdateDeltaState();

    /// 更新预积分模型的协方差矩阵
    void UpdateCov();

    /// 更新雅可比矩阵
    void UpdateJacobian();

    IMU::Ptr last_imu_;     ///< 最新的imu状态
    Vec3d acc_remove_bias_; ///< 去除偏置的加速度计读数
    Vec3d gyr_remove_bias_; ///< 去除偏置的陀螺仪读数
    double last_stamp_;     ///< 预积分系统时间戳
    double dt_;             ///< 每次积分时的子时间
    double integrate_time_; ///< 积分时间

    Mat3d last_dR_; ///< 预积分之前的dr
    SO3d delta_dr_; ///< 旋转部分的更新量
    Mat3d hat_acc_; ///< hat acc_remove_bias的值

    SO3d dR_;        ///< 预积分旋转
    Vec3d dv_;       ///< 预积分速度
    Vec3d dp_;       ///< 预积分位移
    Vec3d acc_bias_; ///< i时刻的加速度计偏置
    Vec3d gyr_bias_; ///< i时刻的陀螺仪偏置

    Mat9d cov_;      ///< 预积分模型协方差矩阵
    Mat6d bias_cov_; ///< 偏置噪声协方差矩阵

    /// 预积分观测与偏置的雅可比
    Mat3d dr_dbg_;
    Mat3d dv_dba_;
    Mat3d dv_dbg_;
    Mat3d dp_dba_;
    Mat3d dp_dbg_;

    Options options_;
};

NAMESPACE_END