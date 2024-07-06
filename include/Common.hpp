#pragma once

#include <list>
#include <memory>

#include <Eigen/Dense>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sophus/se3.hpp>

#define NAMESPACE_BEGIN namespace inc_ndt_lins {
#define NAMESPACE_END }

NAMESPACE_BEGIN

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef Sophus::SE3d SE3d;
typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector3i Vec3i;
typedef g2o::SparseOptimizer Optimizer;
typedef g2o::BlockSolverX BlockSolver;
typedef g2o::LinearSolverEigen<BlockSolver::PoseMatrixType> LinearSolver;
typedef g2o::OptimizationAlgorithmLevenberg Levenberg;
typedef sensor_msgs::msg::PointCloud2 PointCloudMsg;
typedef PointCloudMsg::SharedPtr PointCloudPtr;

struct hash_vec3i {
    std::size_t operator()(const Vec3i &v) const {
        return std::size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
    }
};

Vec3d to_vec3d(const PointT &p);

/**
 * @brief 计算Eigen向量的均值和协方差函数模版
 *
 * @tparam DataType     数据类型
 * @tparam ContentType  输入的内容类型
 * @tparam CovType      协方差类型
 * @param v             输入数据
 * @param mean          输出的均值
 * @param cov           输出的协方差
 * @param f             输入的内容到数据的映射函数
 */
template <typename DataType, typename ContentType, typename CovType>
void ComputeMeanAndCov(const std::vector<ContentType> &v, DataType &mean, CovType &cov,
                       const std::function<DataType(const ContentType &)> &f) {
    std::vector<DataType> datas;
    datas.reserve(v.size());

    DataType u = DataType::Zero();
    for (int i = 0; i < v.size(); ++i) {
        DataType data = f(v[i]);
        datas.push_back(data);
        u += data;
    }
    u /= v.size();

    CovType sigma = CovType::Zero();
    for (int j = 0; j < v.size(); ++j) {
        DataType diff = datas[j] - u;
        sigma += diff * diff.transpose();
    }
    sigma /= v.size() - 1;

    mean = u;
    cov = sigma;
}

/// 使用体素滤波器，对点云进行滤波
PointCloud::Ptr VoxelCloud(PointCloud::Ptr cloud, float voxel_size = 0.1);

NAMESPACE_END
