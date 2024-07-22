#include "IEKF.h"

NAMESPACE_BEGIN

/**
 * @brief 增量NDT的观测模型
 *
 * @param matrix_func   输入的重要矩阵计算函数
 * @param stamp         输入的laser结束时间Ts
 */
void IEKF::ObserveIncNDT(const ObserveMatrixFunc &matrix_func, const double &stamp, double info_ratio) {
    delta_t_ = stamp - last_stamp_;
    assert(delta_t_ >= 0 && "需要观测时间戳一定要大于上一次最新时间戳");
    last_stamp_ = stamp;

    Mat18d HTVH, Qk, Jk, Pk = P_;
    Vec18d HTVr;

    Sophus::SO3d R0 = x_normal_.Twi_.so3();
    for (int i = 0; i < options_.max_iterations_; ++i) {
        matrix_func(x_normal_.Twi_, HTVH, HTVr);
        HTVH *= info_ratio;
        HTVr *= info_ratio;
        Vec3d delta_theta = (x_normal_.Twi_.so3().inverse() * R0).log();
        Jk = Mat18d::Identity();
        Jk.block<3, 3>(6, 6) = Mat3d::Identity() - 0.5 * Sophus::SO3d::hat(delta_theta);
        Pk = Jk * Pk * Jk.transpose();

        Qk = (Pk.inverse() + HTVH).inverse();
        Vec18d dx = Qk * HTVr;
        x_normal_.Update(dx, stamp);
        if (dx.norm() < options_.quit_eps_)
            break;
    }

    /// 迭代观测收敛
    P_ = (Mat18d::Identity() - Qk * HTVH) * Pk;
    Vec3d delta_theta = (x_normal_.Twi_.so3().inverse() * R0).log();
    Jk = Mat18d::Identity();
    Jk.block<3, 3>(6, 6) = Mat3d::Identity() - 0.5 * Sophus::SO3d::hat(delta_theta);
    P_ = Jk * P_ * Jk.transpose();
    return;
}

/**
 * @brief IEKF的预测模型，与ESKF保持一致
 *
 * @param imu 输入的IMU数据
 */
void IEKF::Predict(const IMU::Ptr &imu) {
    last_imu_ = imu;
    delta_t_ = imu->stamp_ - last_stamp_;
    last_stamp_ = imu->stamp_;
    UpdateNormalState();
    ComputeFJacobian();
    P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * @brief 更新名义状态向量
 *
 */
void IEKF::UpdateNormalState() {
    Mat3d R = x_normal_.Twi_.so3().matrix();
    double dt2 = delta_t_ * delta_t_;
    Vec3d acc = last_imu_->acc_ - options_.a_bias_;
    Vec3d gyr = last_imu_->gyr_ - options_.g_bias_;

    x_normal_.Twi_.translation() += x_normal_.v_ * delta_t_ + 0.5 * (R * acc) * dt2;
    x_normal_.v_ += R * acc * delta_t_;
    if (!options_.remove_gravity_) {
        x_normal_.Twi_.translation() += 0.5 * x_normal_.g_ * dt2;
        x_normal_.v_ += x_normal_.g_ * delta_t_;
    }

    x_normal_.Twi_.so3() *= Sophus::SO3d::exp(gyr * delta_t_);
    x_normal_.stamp_ = last_stamp_;
}

/**
 * @brief 计算imu预测模型的雅可比矩阵
 *
 */
void IEKF::ComputeFJacobian() {
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
 * @brief 构建预测模型的噪声矩阵Q
 *
 */
void IEKF::BuildNoise() {
    const double &ev = options_.acc_var_;
    const double &et = options_.gyr_var_;
    const double &eg = options_.g_bias_var_;
    const double &ea = options_.a_bias_var_;

    double ev2 = ev; //* ev;
    double et2 = et; //* et;
    double eg2 = eg; //* eg;
    double ea2 = ea; //* ea;

    Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

    P_ = Mat18d::Identity() * 1e-4;
}

NAMESPACE_END