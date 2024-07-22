#pragma once

// todo 这里写的ESKF代码部分比较冗余，会与上层系统重合，需要重构

#include <rclcpp/rclcpp.hpp>

#include "Common.hpp"

NAMESPACE_BEGIN

/// 导航状态
struct NavState {
    double stamp_;
    SE3d Twi_;
    Vec3d v_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d g_ = Vec3d::Zero();

    void Update(const Vec18d &dx, double stamp);
};

/// IMU信息
struct IMU {
    typedef std::shared_ptr<IMU> Ptr;
    typedef std::shared_ptr<const IMU> ConstPtr;

    double stamp_; ///< IMU的时间戳信息
    Vec3d acc_;    ///< 三轴加速度
    Vec3d gyr_;    ///< 三轴角速度
};

/// IMU的静态初始化类
struct IMUStaticInit {
    typedef std::shared_ptr<IMUStaticInit> Ptr;

    struct Options {
        bool use_speed_check_ = true; ///< 是否使用速度信息进行静态初始化检查
        int buffer_capicity_ = 1000;  ///< 缓存IMU数据的容量
        double static_time_ = 10;     ///< 静态初始化时间
        double gravity_norm_ = 9.81;  ///< 重力加速度模长
        bool remove_gravity_ = false; ///< IMU数据是否去除重力
    };

    IMUStaticInit(Options options)
        : options_(std::move(options))
        , init_success_(false)
        , is_static_(true)
        , delta_t_(0) {}

    /// 添加IMU信息
    void AddIMU(const IMU::Ptr &imu);

    /// 向IMU静态初始化器中添加速度信息
    void AddSpeed(double speed) {
        if (!options_.use_speed_check_) {
            RCLCPP_WARN(rclcpp::get_logger("inc_ndt_lins"), "请打开静态初始化器的use_speed_check_选项");
            return;
        }
        if (speed != 0) {
            is_static_ = false;
            imu_buffer_.clear();
            first_imu_ = nullptr;
            delta_t_ = 0;
        } else
            is_static_ = true;
    }

    std::vector<IMU::Ptr> imu_buffer_; ///< IMU数据队列
    Options options_;                  ///< IMU静态初始化配置项
    bool init_success_;                ///< IMU是否成功初始化
    bool is_static_;                   ///< 初始化时，是否为静态
    double delta_t_;                   ///< IMU初始化累计时间
    IMU::Ptr first_imu_;               ///< 第一个IMU数据信息

    Vec3d gravity_;  ///< 重力加速度
    Vec3d acc_bias_; ///< 加速度计偏置
    Vec3d gyr_bias_; ///< 陀螺仪偏置
    double sigma_a_; ///< 加速度计标准差
    double sigma_g_; ///< 陀螺仪标准差
};

class LooselyLio;

/// 误差状态卡尔曼求解器
class ESKF {
    friend class LooselyLio;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<ESKF> Ptr;

    /// ESKF配置项
    struct Options {
        Vec3d gravity_;              ///< 重力加速度
        Vec3d g_bias_;               ///< 加速度计偏置
        Vec3d a_bias_;               ///< 陀螺仪偏置
        double sigma_a_;             ///< 加速度计标准差
        double sigma_g_;             ///< 陀螺仪标准差
        double sigma_a_bias_ = 1e-4; ///< 加速度计偏置标准差
        double sigma_g_bias_ = 1e-8; ///< 陀螺仪偏置标准差
        double sigma_laser_t = 0.01; ///< SE3观测模型的位置标准差
        double sigma_laser_r = 0.01; ///< SE3观测模型的旋转标准差
    };

    ESKF(IMUStaticInit::Options options = IMUStaticInit::Options())
        : last_imu_(nullptr)
        , delta_t_(0)
        , last_stamp_(0)
        , last_laser_stamp_(0)
        , F_(Mat18d::Identity())
        , Q_(Mat18d::Zero())
        , Vl_(Mat6d::Zero())
        , Hl_(Mat6_18d::Zero())
        , P_(Mat18d::Zero())
        , first_se3_(true) {
        imu_static_init_ = std::make_shared<IMUStaticInit>(options);
    }

    /// 添加IMU信息
    bool AddIMU(const IMU::Ptr &imu);

    /// 添加laser观测信息
    void AddObserveSE3(const SE3d &Twi_observe, double stamp);

    /// ESKF预测阶段
    void Predict();

    /// 用于最后一个imu数据到laser-end-time的状态预测
    void Predice2stamp(const double &laser_end_time) {
        delta_t_ = laser_end_time - last_stamp_;
        Predict();
    }

    /// 获取状态的加速度信息
    Vec3d GetAccerate() { return last_imu_->acc_; }

    /// ESKF更新阶段
    void UpdateL();

    bool ImuInitSuccess() { return imu_static_init_->init_success_; }

    /// 设置ESKF系统的噪声选项
    void SetNoise(double acc_bias_var, double gyr_bias_var, double laser_p_var, double laser_r_var) {
        options_.sigma_a_bias_ = acc_bias_var;
        options_.sigma_g_bias_ = gyr_bias_var;
        options_.sigma_laser_t = laser_p_var;
        options_.sigma_laser_r = laser_r_var;
    }

    SE3d GetTwi() { return x_normal_.Twi_; }

private:
    /// 初始化优化IMU的属性信息
    void InitIMUAttrib();

    /// ESKF的P矩阵重置阶段
    void Reset(const Vec3d &delta_theta);

    /// 计算运动模型的雅可比矩阵
    void ComputeFJacobian();

    /// 计算laser观测模型的雅可比矩阵
    void ComputeHlJacobian();

    /// 更新名义状态向量
    void UpdateNormalState();

    /// 根据配置信息构建噪声的协方差矩阵
    void BuildNoise();

    IMU::Ptr last_imu_;       ///< 最新的imu数据信息
    double delta_t_;          ///< 预测时间间隔
    double last_stamp_;       ///< ESKF滤波器最新的时间戳信息
    double last_laser_stamp_; ///< 上一次雷达观测数据时间戳
    SE3d last_Twi_estimate_;  ///< 最新的SE3观测信息

    Mat18d F_;    ///< 运动模型雅可比矩阵
    Mat18d Q_;    ///< 运动模型噪声协方差矩阵
    Mat6d Vl_;    ///< laser观测模型协方差矩阵
    Mat6_18d Hl_; ///< laser观测模型雅可比矩阵
    Mat18d P_;    ///< ESKF协方差矩阵

    NavState x_normal_; ///< 名义状态向量
    Vec18d delta_x_;    ///< 误差状态向量

    Options options_; ///< ESKF配置项
    bool first_se3_;  ///< 第一个se3观测信息

    IMUStaticInit::Ptr imu_static_init_; ///< imu的静态初始化器
};

NAMESPACE_END