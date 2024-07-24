#include <execution>
#include <functional>
#include <unordered_set>

#include <g2o/core/robust_kernel_impl.h>
#include <rclcpp/rclcpp.hpp>

#include "IncNdt.h"

NAMESPACE_BEGIN

/**
 * @brief 增量NDT算法构造函数
 *
 * @param options 输入的算法配置参数
 */
IncNdt::IncNdt(Options options)
    : options_(std::move(options)) {
    switch (options_.nearby_type_) {
    case NearbyType::CENTER:
        nearby_keys_.push_back(Vec3i(0, 0, 0));
        break;

    case NearbyType::NEARBY6:
        nearby_keys_.push_back(Vec3i(0, 0, 0));
        nearby_keys_.push_back(Vec3i(0, 0, 1));
        nearby_keys_.push_back(Vec3i(0, 0, -1));
        nearby_keys_.push_back(Vec3i(0, 1, 0));
        nearby_keys_.push_back(Vec3i(0, -1, 0));
        nearby_keys_.push_back(Vec3i(1, 0, 0));
        nearby_keys_.push_back(Vec3i(-1, 0, 0));
        break;

    default:
        RCLCPP_ERROR(rclcpp::get_logger("inc_ndt_lins"), "增量NDT初始化失败，NearbyType不合法！");
        break;
    }
}

/**
 * @brief 向体素中添加点云信息
 *
 * @param cloud 输入的要添加的点云
 */
void IncNdt::AddCloud(PointCloud::Ptr cloud) {
    std::unordered_set<Vec3i, hash_vec3i> active_voxels;

    for (int i = 0; i < cloud->size(); ++i) {
        Vec3d pt = to_vec3d(cloud->points[i]);
        Vec3i key = (pt * options_.voxel_size_inv_).cast<int>();
        auto map_iter = voxels_map_.find(key);
        if (map_iter == voxels_map_.end()) {
            Voxel3d voxel;
            voxel.buffer_.push_back(i);
            active_voxels.insert(key);
            voxels_.push_front({key, voxel});
            voxels_map_.insert({key, voxels_.begin()});
            if (voxels_.size() > options_.voxel_capacity_) {
                voxels_map_.erase(voxels_.back().first);
                voxels_.pop_back();
            }
        } else {
            map_iter->second->second.buffer_.push_back(i);
            active_voxels.insert(map_iter->first);
            voxels_.splice(voxels_.begin(), voxels_, map_iter->second);
        }
    }

    for (const auto &key : active_voxels) {
        Voxel3d &voxel = voxels_map_[key]->second;
        if (voxel.estimated_info_)
            voxel.UpdateAttribute(cloud);
        else
            voxel.InitAttribute(cloud);

        if (!voxel.estimated_info_) {
            voxels_.erase(voxels_map_[key]);
            voxels_map_.erase(key);
        }
    }
}

/**
 * @brief 更新体素的分布属性
 *
 * @param cloud 输入的点云
 */
void IncNdt::Voxel3d::UpdateAttribute(const PointCloud::Ptr &cloud) {
    if (buffer_.size() > 5) {
        Vec3d mean = Vec3d::Zero();
        Mat3d cov = Mat3d::Zero();
        ComputeMeanAndCov<Vec3d, int, Mat3d>(buffer_, mean, cov,
                                             [&](const int &i) -> Vec3d { return to_vec3d(cloud->points[i]); });
        Vec3d new_mean = (last_buffer_size_ * mean_ + buffer_.size() * mean) / (last_buffer_size_ + buffer_.size());
        Mat3d new_cov = last_buffer_size_ * (cov_ + (mean_ - new_mean) * (mean_ - new_mean).transpose()) +
                        buffer_.size() * (cov + (mean - new_mean) * (mean - new_mean).transpose());
        mean_ = new_mean;
        cov_ = new_cov / (last_buffer_size_ + buffer_.size());
        SetInfo();

        last_buffer_size_ = buffer_.size();
    }
    buffer_.clear();
}

/**
 * @brief 初始化点云的分布属性信息
 *
 * @param cloud 输入的点云地图
 */
void IncNdt::Voxel3d::InitAttribute(const PointCloud::Ptr &cloud) {
    if (buffer_.size() > 5) {
        ComputeMeanAndCov<Vec3d, int, Mat3d>(buffer_, mean_, cov_,
                                             [&](const int &i) -> Vec3d { return to_vec3d(cloud->points[i]); });
        SetInfo();

        estimated_info_ = true;
        last_buffer_size_ = buffer_.size();
    }

    buffer_.clear();
}

/**
 * @brief 设置信息矩阵，主要判断sigma协方差是否可逆
 *
 */
void IncNdt::Voxel3d::SetInfo() {
    Eigen::JacobiSVD svd(cov_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vec3d lambda = svd.singularValues();
    if (lambda[1] < lambda[0] * 1e-3)
        lambda[1] = lambda[0] * 1e-3;
    if (lambda[2] < lambda[0] * 1e-3)
        lambda[2] = lambda[0] * 1e-3;

    Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
    info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
}

/**
 * @brief 使用g2o优化增量NDT模型进行配准
 *
 * @param cloud     输入的待配装的点云
 * @param init_pose 输出的配准成功的位姿
 * @return int 输出配准过程中ndt配对成功的点数
 */
int IncNdt::AlignG2O(PointCloud::Ptr cloud, SE3d &init_pose) {
    Optimizer optimizer;
    GaussNewton *gn = new GaussNewton(std::make_unique<BlockSolver>(std::make_unique<LinearSolver>()));
    optimizer.setAlgorithm(gn);

    SE3Vertex *v = new SE3Vertex;
    v->setId(0);
    v->setEstimate(init_pose);
    optimizer.addVertex(v);

    int edge_id = 0;
    std::vector<NDTEdge *> ndt_edges;
    for (auto &pt : cloud->points) {
        Vec3d pb = to_vec3d(pt);
        Vec3d pw = init_pose * pb;
        Vec3i key = (pw * options_.voxel_size_inv_).cast<int>();

        for (const auto n : nearby_keys_) {
            Vec3i new_key = key + n;
            auto map_iter = voxels_map_.find(new_key);
            if (map_iter == voxels_map_.end())
                continue;
            const Voxel3d &voxel = map_iter->second->second;
            if (!voxel.estimated_info_)
                continue;

            NDTEdge *e = new NDTEdge(voxel.mean_);
            e->setId(edge_id++);
            e->setVertex(0, v);
            e->setInformation(voxel.info_);
            e->setMeasurement(pb);
            e->computeError();
            if (e->chi2() > options_.out_lier_th_) {
                delete e;
                continue;
            }
            // g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            // rk->setDelta(options_.out_lier_th_);
            // e->setRobustKernel(rk);
            optimizer.addEdge(e);
            ndt_edges.push_back(e);
        }
    }

    optimizer.setVerbose(true);

    int ninlier_num = 0;

    for (int i = 0; i < 3; i++) {
        ninlier_num = 0;
        optimizer.initializeOptimization(0);
        optimizer.optimize(1);
        for (auto &e : ndt_edges) {
            e->computeError();
            if (e->chi2() < options_.out_lier_th_) {
                e->setLevel(0);
                ++ninlier_num;
            } else {
                e->setLevel(1);
            }
        }
    }

    init_pose = v->estimate();
    return ninlier_num;
}

/// 获取点云中的重要矩阵信息
void IncNdt::GetHessian(const PointCloud::Ptr &cloud, const SE3d &Twl, Mat18d &HTVH, Vec18d &HTVr) {
    int pts_num = 1;
    if (options_.nearby_type_ == NearbyType::NEARBY6)
        pts_num = 7;

    std::vector<int> indices(cloud->size());
    std::vector<bool> point_valid(cloud->size() * pts_num, true);

    /// 保存不同点的雅可比矩阵和残差
    std::vector<Mat3_6d, Eigen::aligned_allocator<Mat3_6d>> jacobians(cloud->size() * pts_num, Mat3_6d::Zero());
    std::vector<Vec3d, Eigen::aligned_allocator<Vec3d>> residuals(cloud->size() * pts_num, Vec3d::Zero());
    std::vector<Mat3d, Eigen::aligned_allocator<Mat3d>> infos(cloud->size() * pts_num, Mat3d::Zero());

    std::for_each(indices.begin(), indices.end(), [id = 0](int &idx) mutable { idx = id++; });
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
    // std::for_each(indices.begin(), indices.end(), [&](const int &idx) {
        Vec3d pl = to_vec3d(cloud->points[idx]);
        Vec3d pw = Twl * pl;

        Vec3i key = (pw * options_.voxel_size_inv_).cast<int>();
        for (int id = 0; id < pts_num; ++id) {
            int total_id = idx * pts_num + id;
            const auto &n = nearby_keys_[id];
            Vec3i new_key = key + n;
            auto map_iter = voxels_map_.find(new_key);
            if (map_iter == voxels_map_.end()) {
                point_valid[total_id] = false;
                continue;
            }
            const Voxel3d &voxel = map_iter->second->second;
            if (!voxel.estimated_info_) {
                point_valid[total_id] = false;
                continue;
            }

            Vec3d res = pw - voxel.mean_;
            double chi2 = res.transpose() * voxel.info_ * res;
            if (std::isnan(chi2) || chi2 > options_.out_lier_th_) {
                point_valid[total_id] = false;
                continue;
            }
            Mat3d dr_dR = -Twl.so3().matrix() * Sophus::SO3d::hat(pl);
            Mat3d dr_dt = Mat3d::Identity();
            jacobians[total_id].block<3, 3>(0, 0) = dr_dt;
            jacobians[total_id].block<3, 3>(0, 3) = dr_dR;

            residuals[total_id] = res;
            infos[total_id] = voxel.info_;
        }
    });

    HTVH.setZero();
    HTVr.setZero();

    for (int i = 0; i < cloud->size() * pts_num; ++i) {
        if (!point_valid[i])
            continue;

        // todo 这里是否需要乘dx/dδx?
        Mat3_18d J = Mat3_18d::Zero();
        J.block<3, 3>(0, 0) = jacobians[i].block<3, 3>(0, 0);
        J.block<3, 3>(0, 6) = jacobians[i].block<3, 3>(0, 3);

        HTVH += J.transpose() * infos[i] * J;
        HTVr += -J.transpose() * infos[i] * residuals[i];
    }
}

NAMESPACE_END