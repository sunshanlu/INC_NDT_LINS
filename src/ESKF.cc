#include <rclcpp/rclcpp.hpp>

#include "ESKF.h"

NAMESPACE_BEGIN

/// 添加IMU信息
void ESKF::AddIMU(const IMU::Ptr &imu) {
    double dt = imu->stamp_ - last_stamp_;
    if (dt < 0) {
        RCLCPP_INFO(rclcpp::get_logger("inc_ndt_lins"), "imu数据出现回退！");
        return;
    }
    if (!imu_static_init_->init_success_) {
        imu_static_init_->AddIMU(imu);
        if (imu_static_init_->init_success_)
            InitIMUAttrib();
    }

    if (!first_se3_) {
        last_stamp_ = imu->stamp_;
        delta_t_ = dt;
        last_imu_ = imu;
        Predict();
    }
}

/// 添加laser观测信息
void ESKF::AddObserveSE3(const SE3d &Twi_observe, double stamp) {
    double dt = stamp - last_stamp_;
    if (dt < 0) {
        RCLCPP_INFO(rclcpp::get_logger("inc_ndt_lins"), "观测信息出现回退");
        return;
    }

    if (first_se3_) {
        first_se3_ = false;
        x_normal_.Twi_ = Twi_observe;
    }

    if (!first_se3_ && imu_static_init_->init_success_) {
        last_stamp_ = stamp;
        delta_t_ = dt;
        last_Twi_estimate_ = Twi_observe;
        UpdateL();
    }
}

/**
 * @brief 根据最新的imu和名义状态计算F矩阵
 * @details
 *      1. imu数据来的时候，会调用
 *      2. 使用的是更新后的名义状态量计算F
 */
void ESKF::ComputeFJacobian() {
    F_ = Mat18d::Identity();
    Mat3d R = x_normal_.Twi_.so3().matrix();

    F_.block<3, 3>(0, 3) = delta_t_ * Mat3d::Identity();
    F_.block<3, 3>(3, 6) = R * Sophus::SO3d::hat(last_imu_->acc_ - options_.a_bias_) * delta_t_;
    F_.block<3, 3>(3, 12) = -R * delta_t_;
    F_.block<3, 3>(3, 15) = Mat3d::Identity() * delta_t_;
    F_.block<3, 3>(6, 6) = Sophus::SO3d::exp(-(last_imu_->gyr_ - options_.g_bias_) * delta_t_).matrix();
    F_.block<3, 3>(6, 9) = -Mat3d::Identity() * delta_t_;
}

/**
 * @brief 根据链式求导，求解Hl的雅可比矩阵
 * @details
 *      z对delta_x（误差状态）求导 = (z 对 x（真值）求导) * (x（真值）对delta_x（误差状态）求导)
 */
void ESKF::ComputeHlJacobian() {
    Hl_ = Mat6_18d::Zero();
    Vec3d theta = x_normal_.Twi_.so3().log();

    Hl_.block<3, 3>(0, 0) = Mat3d::Identity();
    Hl_.block<3, 3>(3, 6) = Sophus::SO3d::leftJacobianInverse(-theta);
}

/**
 * @brief 根据输入的imu数据，更新名义状态量
 * @details
 *      1. 使用反向欧拉法进行名义状态向量的更新
 */
void ESKF::UpdateNormalState() {
    Mat3d R = x_normal_.Twi_.so3().matrix();
    double dt2 = delta_t_ * delta_t_;
    Vec3d acc = last_imu_->acc_ - options_.a_bias_;
    Vec3d gyr = last_imu_->gyr_ - options_.g_bias_;

    x_normal_.Twi_.translation() += x_normal_.v_ * delta_t_ + 0.5 * (R * acc) * dt2 + 0.5 * x_normal_.g_ * dt2;
    x_normal_.v_ += R * acc * delta_t_ + x_normal_.g_ * delta_t_;
    x_normal_.Twi_.so3() *= x_normal_.Twi_.so3() * Sophus::SO3d::exp(gyr * delta_t_);
    x_normal_.stamp_ = last_stamp_;
}

/**
 * @brief ESKF滤波器的重置函数，重置P和误差状态
 *
 */
void ESKF::Reset(const Vec3d &delta_theta) {
    delta_x_ = Vec18d::Zero();

    Mat3d J_theta = Mat3d::Identity() - 0.5 * Sophus::SO3d::hat(delta_theta);
    Mat18d J_k = Mat18d::Identity();
    J_k.block<3, 3>(6, 6) = J_theta;
    P_ = J_k * P_ * J_k.transpose();
}

/**
 * @brief ESKF系统的预测阶段
 * @details
 *      1. 更新名义状态向量，以便F矩阵计算
 *      2. 计算F矩阵
 *      3. 更新协方差矩阵P
 */
void ESKF::Predict() {
    UpdateNormalState();
    ComputeFJacobian();
    P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * @brief ESKF系统的更新阶段
 * @details
 *      1. 计算Hl雅可比矩阵（观测模型）
 *      2. 计算K矩阵，卡尔曼增益
 *      3. 计算最终的更新量dx
 *      4. 将更新量dx放到名义状态向量中
 *      5. 更新协方差矩阵P
 *      6. 重置整个系统
 */
void ESKF::UpdateL() {
    ComputeHlJacobian();

    Mat18_6d K = P_ * Hl_.transpose() * (Hl_ * P_ * Hl_.transpose() + Vl_);

    Vec6d z = Vec6d::Zero(), h_x = Vec6d::Zero();
    z.head<3>() = last_Twi_estimate_.translation();
    z.tail<3>() = last_Twi_estimate_.so3().log();
    h_x.head<3>() = x_normal_.Twi_.translation();
    h_x.tail<3>() = x_normal_.Twi_.so3().log();

    Vec18d dx = K * (z - h_x);
    x_normal_.Update(dx, last_stamp_);

    P_ = (Mat18d::Identity() - K * Hl_) * P_;
    Reset(dx.block<3, 1>(6, 0));
}

/**
 * @brief 更新导航状态量
 *
 * @param dx    输入的更新量
 * @param stamp 输入的时间戳
 */
void NavState::Update(const Vec18d &dx, double stamp) {
    Twi_.translation() += dx.block<3, 1>(0, 0);
    v_ += dx.block<3, 1>(3, 0);
    Twi_.so3() *= Sophus::SO3d::exp(dx.block<3, 1>(6, 0));
    bg_ += dx.block<3, 1>(9, 0);
    ba_ += dx.block<3, 1>(12, 0);
    g_ += dx.block<3, 1>(15, 0);

    stamp_ = std::move(stamp);
}

/**
 * @brief 添加IMU信息
 *
 * @param imu 输入的IMU信息
 */
void IMUStaticInit::AddIMU(const IMU::Ptr &imu) {
    if (options_.use_speed_check_)
        if (!is_static_)
            return;

    if (!first_imu_)
        first_imu_ = imu;

    imu_buffer_.push_back(imu);
    delta_t_ = imu->stamp_ - first_imu_->stamp_;

    if (imu_buffer_.size() >= options_.buffer_capicity_ || delta_t_ >= options_.static_time_) {
        Vec3d acc_mean = Vec3d::Zero(), gyr_mean = Vec3d::Zero();
        Mat3d acc_cov = Mat3d::Zero(), gyr_cov = Mat3d::Zero();
        ComputeMeanAndCov<Vec3d, IMU::Ptr, Mat3d>(imu_buffer_, acc_mean, acc_cov,
                                                  [](const IMU::Ptr &imu) -> Vec3d { return imu->acc_; });
        ComputeMeanAndCov<Vec3d, IMU::Ptr, Mat3d>(imu_buffer_, gyr_mean, gyr_cov,
                                                  [](const IMU::Ptr &imu) -> Vec3d { return imu->gyr_; });

        gravity_ = acc_mean / acc_mean.norm() * options_.gravity_norm_;
        acc_bias_ = acc_mean - gravity_;
        gyr_bias_ = gyr_mean;
        sigma_a_ = std::sqrt(acc_cov(0, 0));
        sigma_g_ = std::sqrt(gyr_cov(0, 0));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("inc_ndt_lins"), "Init success!");

        // clang-format off
        RCLCPP_INFO_STREAM(rclcpp::get_logger("inc_ndt_lins"),
            "gravity: " << gravity_.transpose() << 
            " bias_acc: " << acc_bias_.transpose() << 
            " bias_gyr: " << gyr_bias_.transpose() << 
            " sigma_a: " << sigma_a_ << " sigma_g: " << sigma_g_
        );
        // clang-format on
    }
}

/**
 * @brief 初始化IMU相关属性
 *
 */
void ESKF::InitIMUAttrib() {
    /// ESKF配置选项初始化
    options_.gravity_ = imu_static_init_->gravity_;
    options_.a_bias_ = imu_static_init_->acc_bias_;
    options_.g_bias_ = imu_static_init_->gyr_bias_;
    options_.sigma_a_ = imu_static_init_->sigma_a_;
    options_.sigma_g_ = imu_static_init_->sigma_g_;

    /// ESKF初始名义状态向量初始化
    x_normal_.ba_ = options_.a_bias_;
    x_normal_.bg_ = options_.g_bias_;
    x_normal_.g_ = options_.gravity_;

    BuildNoise();
}

/**
 * @brief 构建ESKF的噪声协方差矩阵
 *
 */
void ESKF::BuildNoise() {
    const double &ev = options_.sigma_a_;
    const double &et = options_.sigma_g_;
    const double &eg = options_.sigma_g_bias_;
    const double &ea = options_.sigma_a_bias_;

    double ev2 = ev; //* ev;
    double et2 = et; //* et;
    double eg2 = eg; //* eg;
    double ea2 = ea; //* ea;

    Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

    double lp2 = options_.sigma_laser_t * options_.sigma_laser_t;
    double la2 = options_.sigma_laser_r * options_.sigma_laser_r;

    Vl_.diagonal() << lp2, lp2, lp2, la2, la2, la2;
}

NAMESPACE_END
