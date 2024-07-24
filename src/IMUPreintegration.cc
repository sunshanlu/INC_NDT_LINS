#include "IMUPreintegration.h"

NAMESPACE_BEGIN

/**
 * @brief 预积分系统的积分api
 * @details
 *      1. 更新积分状态，dr、dv和dp
 *      2. 更新积分模型的雅可比矩阵
 *      3. 更新积分状态对偏置ba、bg的雅可比矩阵
 *
 * @note 注意更新状态时的顺序问题
 */
void IMUPreintegration::Integrate(const IMU::Ptr &imu, double stamp) {
    last_imu_ = imu;

    dt_ = stamp - last_stamp_;
    integrate_time_ += dt_;
    last_stamp_ = stamp;
    acc_remove_bias_ = last_imu_->acc_ - acc_bias_;
    gyr_remove_bias_ = last_imu_->gyr_ - gyr_bias_;
    hat_acc_ = SO3d::hat(acc_remove_bias_);

    UpdateDeltaState();
    UpdateCov();
    UpdateJacobian();
}

/**
 * @brief 状态积分
 * @details
 *      1. 保留last_dR,用于后续的协方差和雅可比更新
 *      2. dp和dv需要注意更新顺序
 */
void IMUPreintegration::UpdateDeltaState() {
    last_dR_ = dR_;
    dp_ = dp_ + dv_ * dt_ + 0.5 * last_dR_.matrix() * acc_remove_bias_ * dt_ * dt_;
    dv_ = dv_ + last_dR_ * acc_remove_bias_ * dt_;
    delta_dr_ = SO3d::exp(gyr_remove_bias_ * dt_);
    dR_ = dR_ * delta_dr_;
}

/**
 * @brief 更新预积分模型的协方差矩阵
 *
 */
void IMUPreintegration::UpdateCov() {
    Mat9d A = Mat9d::Identity();
    Mat9_6d B = Mat9_6d::Zero();

    A.block<3, 3>(0, 0) = delta_dr_.inverse().matrix();
    A.block<3, 3>(3, 0) = -last_dR_.matrix() * hat_acc_ * dt_;
    A.block<3, 3>(6, 0) = -0.5 * last_dR_.matrix() * hat_acc_ * dt_ * dt_;
    A.block<3, 3>(6, 3) = Mat3d::Identity() * dt_;

    B.block<3, 3>(0, 0) = SO3d::leftJacobian(-gyr_remove_bias_ * dt_) * dt_;
    B.block<3, 3>(3, 3) = last_dR_.matrix() * dt_;
    B.block<3, 3>(6, 3) = 0.5 * last_dR_.matrix() * dt_ * dt_;

    cov_ = A * cov_ * A.transpose() + B * bias_cov_ * B.transpose();
}

/**
 * @brief 更新积分状态对偏置的雅可比矩阵
 *
 */
void IMUPreintegration::UpdateJacobian() {
    dp_dbg_ = dp_dbg_ + dv_dbg_ * dt_ - 0.5 * last_dR_.matrix() * hat_acc_ * dr_dbg_ * dt_ * dt_;
    dp_dba_ = dp_dba_ + dv_dba_ * dt_ - 0.5 * last_dR_.matrix() * dt_ * dt_;
    dv_dbg_ = dv_dbg_ - last_dR_.matrix() * hat_acc_ * dr_dbg_ * dt_;
    dv_dba_ = dv_dba_ - last_dR_.matrix() * dt_;
    dr_dbg_ = delta_dr_.inverse().matrix() * dr_dbg_ - SO3d::leftJacobian(-gyr_remove_bias_ * dt_) * dt_;
}

/**
 * @brief 输入i时刻状态，输出积分时刻的状态
 *
 * @param state_i   输入的i时刻的状态
 * @return NavState 输出的i+dt时刻的积分状态
 */
NavState IMUPreintegration::Predict(const NavState &state_i) {
    Mat3d Ri = state_i.Twi_.so3().matrix();
    const Vec3d &vi = state_i.v_;
    const Vec3d &pi = state_i.Twi_.translation();

    NavState new_state;
    new_state.stamp_ = state_i.stamp_ + integrate_time_;
    new_state.Twi_.so3() = state_i.Twi_.so3() * dR_;
    new_state.v_ = Ri * dv_ + vi;
    if (!options_.remove_gravity_)
        new_state.v_ += options_.gravity_ * integrate_time_;

    new_state.Twi_.translation() = Ri * dp_ + vi * integrate_time_ + pi;
    if (!options_.remove_gravity_)
        new_state.Twi_.translation() += 0.5 * options_.gravity_ * integrate_time_ * integrate_time_;

    new_state.ba_ = state_i.ba_;
    new_state.bg_ = state_i.bg_;
    new_state.g_ = state_i.g_;
    return new_state;
}

/**
 * @brief 获取预积分模型的信息矩阵
 *
 * @return Mat9d 输出的信息矩阵
 */
Mat9d IMUPreintegration::GetInformation() {
    Eigen::JacobiSVD svd(cov_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vec9d lambda = svd.singularValues();
    for (int i = 0; i < 8; ++i) {
        if (lambda[i + 1] < lambda[i] * 1e-3)
            lambda[i + 1] = lambda[i] * 1e-3;
    }

    for (int i = 0; i < 9; ++i)
        lambda[i] = 1 / lambda[i];

    Mat9d inv_lambda = lambda.asDiagonal();
    return svd.matrixV() * inv_lambda * svd.matrixU().transpose();
}

NAMESPACE_END