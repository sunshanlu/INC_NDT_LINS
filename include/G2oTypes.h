#pragma once

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>

#include "Common.hpp"
#include "IMUPreintegration.h"
#include "IncNdt.h"

NAMESPACE_BEGIN

/// 预积分边
class IntegEdge : public g2o::BaseMultiEdge<9, Vec9d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IntegEdge(IMUPreintegration::Ptr preint)
        : preint_(std::move(preint)) {
        resize(6);
        setInformation(preint_->GetInformation());
        remove_gravity_ = preint_->options_.remove_gravity_;
        gravity_ = preint_->options_.gravity_;
        dt_ = preint_->integrate_time_;
    }

    void computeError() override;

    void linearizeOplus() override;

    bool write(std::ostream &) const override { return false; }

    bool read(std::istream &) override { return false; }

    /// 获取预积分因子的海塞矩阵
    Mat24d GetHessian() {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J = Eigen::Matrix<double, 9, 24>::Zero();
        J.block<9, 6>(0, 0) = _jacobianOplus[0];  ///< de / dTi
        J.block<9, 3>(0, 6) = _jacobianOplus[1];  ///< de / dvi
        J.block<9, 3>(0, 9) - _jacobianOplus[2];  ///< de / dbgi
        J.block<9, 3>(0, 12) = _jacobianOplus[3]; ///< de / dbai
        J.block<9, 6>(0, 15) = _jacobianOplus[4]; ///< de / dTj
        J.block<9, 3>(0, 21) = _jacobianOplus[5]; ///< de / dvj
        return J.transpose() * _information * J;
    }

private:
    IMUPreintegration::Ptr preint_; ///< 预积分器
    bool remove_gravity_;           ///< 是否移除重力
    Vec3d gravity_;                 ///< 重力
    double dt_;                     ///< 积分时间

    /// 计算需要的状态量，由computeError提供
    Vec3d bgi_, bai_;    ///< 偏置
    Vec3d vi_, vj_;      ///< 速度
    Vec3d pi_, pj_;      ///< 位置
    SO3d Ri_, Rj_;       ///< 旋转
    Vec3d eR_, ev_, ep_; ///< 误差
};

/// 先验边
class PriorEdge : public g2o::BaseMultiEdge<15, Vec15d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PriorEdge(NavState xi)
        : xi_(std::move(xi)) {
        resize(4);
    }

    void computeError() override;

    void linearizeOplus() override;

    bool write(std::ostream &) const override { return false; }

    bool read(std::istream &) override { return false; }

    /// 获取先验因子的海塞矩阵
    Mat15d GetHessian() {
        linearizeOplus();
        Mat15d J = Mat15d::Zero();
        J.block<15, 6>(0, 0) = _jacobianOplus[0];
        J.block<15, 3>(0, 6) = _jacobianOplus[1];
        J.block<15, 3>(0, 9) = _jacobianOplus[2];
        J.block<15, 3>(0, 12) = _jacobianOplus[3];
        return J.transpose() * _information * J;
    }

private:
    NavState xi_; ///< i时刻的观测

    Vec3d eR_; ///< 旋转部分残差，求雅可比使用
};

/// 三维顶点边
class Vec3dVertex : public g2o::BaseVertex<3, Vec3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool write(std::ostream &) const override { return false; }

    bool read(std::istream &) override { return false; }

    void setToOriginImpl() override { _estimate = Vec3d::Zero(); }

    /// 更新顶点状态
    void oplusImpl(const double *update) override {
        Vec3d update_vec;
        update_vec << update[0], update[1], update[2];
        _estimate += update_vec;
    }
};

typedef Vec3dVertex AccBiasVertex;
typedef Vec3dVertex GyrBiasVertex;
typedef Vec3dVertex VelocityVertex;

/// 三维随机游走边
class Vec3dEdge : public g2o::BaseBinaryEdge<3, Vec3d, Vec3dVertex, Vec3dVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool write(std::ostream &) const override { return false; }

    bool read(std::istream &) override { return false; }

    void computeError() override {
        auto xi = dynamic_cast<Vec3dVertex *>(_vertices[0]);
        auto xj = dynamic_cast<Vec3dVertex *>(_vertices[1]);
        _error = xi->estimate() - xj->estimate();
    }

    void linearizeOplus() override {
        _jacobianOplusXi = Mat3d::Identity();
        _jacobianOplusXj = -Mat3d::Identity();
    }

    /// 获取随机游走因子的海塞矩阵
    Mat6d GetHessian() {
        linearizeOplus();
        Mat3_6d J = Mat3_6d::Zero();
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;

        return J.transpose() * _information * J;
    }
};

typedef Vec3dEdge AccBiasEdge;
typedef Vec3dEdge GyrBiasEdge;

/// SE3边，在紧耦合模型中，为gnss或lo的位姿输出
class SE3Edge : public g2o::BaseUnaryEdge<6, SE3d, SE3Vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool write(std::ostream &) const override { return false; }

    bool read(std::istream &) override { return false; }

    void computeError() override;

    void linearizeOplus() override;

    /// 获取SE3观测因子的海塞矩阵
    Mat6d GetHessian() {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * _information * _jacobianOplusXi;
    }

private:
    Vec3d eR_;
};

typedef SE3Edge NdtObserveEdge;
typedef SE3Edge GNSSObserveEdge;

NAMESPACE_END