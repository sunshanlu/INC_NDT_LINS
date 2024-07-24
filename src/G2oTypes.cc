#include "G2oTypes.h"

NAMESPACE_BEGIN

/**
 * @brief 预积分边误差求解，eR ev vp
 *
 */
void IntegEdge::computeError() {
    auto vTi = dynamic_cast<SE3Vertex *>(_vertices[0]);
    auto vvi = dynamic_cast<VelocityVertex *>(_vertices[1]);
    auto vbgi = dynamic_cast<GyrBiasVertex *>(_vertices[2]);
    auto vbai = dynamic_cast<AccBiasVertex *>(_vertices[3]);
    auto vTj = dynamic_cast<SE3Vertex *>(_vertices[4]);
    auto vvj = dynamic_cast<VelocityVertex *>(_vertices[5]);

    bgi_ = vbgi->estimate();
    bai_ = vbai->estimate();
    Ri_ = vTi->estimate().so3();
    Rj_ = vTj->estimate().so3();
    vi_ = vvi->estimate();
    vj_ = vvj->estimate();
    pi_ = vTi->estimate().translation();
    pj_ = vTj->estimate().translation();

    SO3d dR = preint_->GetDeltaR(bgi_);
    Vec3d dv = preint_->GetDeltaV(bgi_, bai_);
    Vec3d dp = preint_->GetDeltaP(bgi_, bai_);

    eR_ = (dR.inverse() * (Ri_.inverse() * Rj_)).log();
    ev_ = Ri_.inverse().matrix() * (vj_ - vi_) - dv;
    if (!remove_gravity_)
        ev_ -= Ri_.inverse().matrix() * gravity_ * dt_;

    ep_ = Ri_.inverse() * (pj_ - pi_ - vi_ * dt_) - dp;
    if (!remove_gravity_)
        ep_ -= Ri_.inverse() * gravity_ * dt_ * dt_ * 0.5;

    _error << eR_, ev_, ep_;
}

/**
 * @brief 求解残差的线性化雅可比
 *
 */
void IntegEdge::linearizeOplus() {

    Mat3d jr_inv_eR = SO3d::leftJacobianInverse(-eR_);

    /// 误差对Ti的雅可比
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3, 3>(0, 3) = -jr_inv_eR * Rj_.inverse().matrix() * Ri_.matrix(); ///< eR / Ri
    _jacobianOplus[0].block<3, 3>(6, 0) = -Ri_.inverse().matrix();                            ///< ep / pi

    if (remove_gravity_) {
        _jacobianOplus[0].block<3, 3>(3, 3) = SO3d::hat(Ri_.inverse() * (vj_ - vi_));             ///< ev / Ri
        _jacobianOplus[0].block<3, 3>(6, 3) = SO3d::hat(Ri_.inverse() * (pj_ - pi_ - vi_ * dt_)); ///< ep / Ri
    } else {
        _jacobianOplus[0].block<3, 3>(3, 3) = SO3d::hat(Ri_.inverse() * (vj_ - vi_ - gravity_ * dt_)); ///< ev / Ri
        _jacobianOplus[0].block<3, 3>(6, 3) =
            SO3d::hat(Ri_.inverse() * (pj_ - pi_ - vi_ * dt_ - 0.5 * gravity_ * dt_ * dt_)); ///< ep / Ri
    }

    /// 误差对vi的雅可比
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(3, 0) = -Ri_.inverse().matrix();       ///< ev / vi
    _jacobianOplus[1].block<3, 3>(6, 0) = -Ri_.inverse().matrix() * dt_; ///< ep / vi

    /// 误差对bgi的雅可比
    Vec3d delta_bgi = bgi_ - preint_->gyr_bias_;
    Mat3d exp_eR_inv = SO3d::exp(eR_).inverse().matrix();
    Mat3d jr_dR_dbg = SO3d::leftJacobianInverse(-preint_->dr_dbg_ * delta_bgi);
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(0, 0) = -jr_inv_eR * exp_eR_inv * jr_dR_dbg * preint_->dr_dbg_; ///< eR / bgi
    _jacobianOplus[2].block<3, 3>(3, 0) = -preint_->dv_dbg_;                                      ///< ev / bgi
    _jacobianOplus[2].block<3, 3>(6, 0) = -preint_->dp_dbg_;                                      ///< ep / bgi

    /// 误差对bai的雅可比
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(3, 0) = -preint_->dv_dba_; ///< ev / bai
    _jacobianOplus[3].block<3, 3>(6, 0) = -preint_->dp_dba_; ///< ep / bai

    /// 误差对Tj的雅可比
    _jacobianOplus[4].setZero();
    _jacobianOplus[4].block<3, 3>(0, 3) = jr_inv_eR;              ///< eR / Rj
    _jacobianOplus[4].block<3, 3>(6, 0) = Ri_.inverse().matrix(); ///< ep / pj

    /// 误差对vj的雅可比
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3, 3>(3, 0) = Ri_.inverse().matrix(); ///< ev / vj
}

/**
 * @brief 先验边，计算误差15维
 *
 */
void PriorEdge::computeError() {
    auto vTi = dynamic_cast<SE3Vertex *>(_vertices[0]);
    auto vvi = dynamic_cast<VelocityVertex *>(_vertices[1]);
    auto vbgi = dynamic_cast<GyrBiasVertex *>(_vertices[2]);
    auto vbai = dynamic_cast<AccBiasVertex *>(_vertices[3]);

    auto Ri = vTi->estimate().so3();
    eR_ = (xi_.Twi_.so3().inverse() * Ri).log();
    Vec3d ev = vvi->estimate() - xi_.v_;
    Vec3d ep = vTi->estimate().translation() - xi_.Twi_.translation();
    Vec3d ebg = vbgi->estimate() - xi_.bg_;
    Vec3d eba = vbai->estimate() - xi_.ba_;

    _error << ep, eR_, ev, ebg, eba;
}

/**
 * @brief 先验边线性化
 *
 */
void PriorEdge::linearizeOplus() {
    _jacobianOplus[0].setZero();
    _jacobianOplus[0].block<3, 3>(0, 0) = Mat3d::Identity();
    _jacobianOplus[0].block<3, 3>(3, 3) = SO3d::leftJacobianInverse(-eR_);

    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(6, 0) = Mat3d::Identity();

    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(9, 0) = Mat3d::Identity();

    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(12, 0) = Mat3d::Identity();
}

/**
 * @brief 对SE3边，计算误差
 *
 */
void SE3Edge::computeError() {
    auto vTj = dynamic_cast<SE3Vertex *>(_vertices[0]);
    eR_ = (_measurement.so3().inverse() * vTj->estimate().so3()).log();
    Vec3d ep = vTj->estimate().translation() - _measurement.translation();
    _error << ep, eR_;
}

/**
 * @brief 对SE3边，计算线性雅可比
 *
 */
void SE3Edge::linearizeOplus() {
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<3, 3>(0, 0) = Mat3d::Identity();
    _jacobianOplusXi.block<3, 3>(3, 3) = SO3d::leftJacobianInverse(-eR_);
}

NAMESPACE_END
