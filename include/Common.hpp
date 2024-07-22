#pragma once

#include <list>
#include <memory>

#include <Eigen/Dense>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sophus/se3.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>

#define NAMESPACE_BEGIN namespace inc_ndt_lins {
#define NAMESPACE_END }

NAMESPACE_BEGIN

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef Sophus::SE3d SE3d;
typedef Sophus::SO3d SO3d;
typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Matrix<double, 9, 1> Vec9d;
typedef Eigen::Matrix<double, 9, 9> Mat9d;
typedef Eigen::Matrix<double, 15, 1> Vec15d;
typedef Eigen::Matrix<double, 15, 15> Mat15d;
typedef Eigen::Matrix<double, 9, 6> Mat9_6d;
typedef Eigen::Matrix<double, 18, 18> Mat18d;
typedef Eigen::Matrix<double, 18, 1> Vec18d;
typedef Eigen::Matrix<double, 6, 6> Mat6d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, 6, 18> Mat6_18d;
typedef Eigen::Matrix<double, 18, 6> Mat18_6d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 18> MatX_18d;
typedef Eigen::Matrix<double, 18, Eigen::Dynamic> Mat18_Xd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecXd;
typedef Eigen::Matrix<double, 3, 6> Mat3_6d;
typedef Eigen::Matrix<double, 3, 18> Mat3_18d;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::MatrixXd MatXd;
typedef g2o::SparseOptimizer Optimizer;
typedef g2o::BlockSolverX BlockSolver;
typedef g2o::LinearSolverEigen<BlockSolver::PoseMatrixType> LinearSolver;
typedef g2o::OptimizationAlgorithmLevenberg Levenberg;
typedef sensor_msgs::msg::PointCloud2 PointCloudMsg;
typedef sensor_msgs::msg::Imu ImuMsg;
typedef PointCloudMsg::SharedPtr PointCloudPtr;
typedef livox_ros_driver2::msg::CustomMsg LivoxCloud;

struct hash_vec3i {
    std::size_t operator()(const Vec3i &v) const {
        return std::size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
    }
};

// clang-format off
template <typename PointType> 
Vec3d to_vec3d(const PointType &p) { 
    return Vec3d(p.x, p.y, p.z); 
}

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
// clang-format on

/// 使用体素滤波器，对点云进行滤波
PointCloud::Ptr VoxelCloud(PointCloud::Ptr cloud, float voxel_size = 0.1);

/// @brief 全量信息点
struct FullPoint {
    PCL_ADD_POINT4D

    float range = 0;
    float radius = 0;
    std::uint8_t intensity = 0;
    std::uint8_t ring = 0;
    std::uint8_t angle = 0;
    double time = 0;
    float height = 0;

    FullPoint() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

NAMESPACE_END

/// velodyne的点云点类型
namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D
    float intensity;
    double time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace velodyne_ros

/// ouster的点云点类型
namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t ambient;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros

/// 向pcl注册点云结构信息
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(inc_ndt_lins::FullPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, range, range)
                                  (float, radius, radius)
                                  (std::uint8_t, intensity, intensity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint8_t, angle,angle)
                                  (double, time, time)
                                  (float, height, height))

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, time, time)
                                  (std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint16_t, ambient, ambient)
                                  (std::uint32_t, range, range)
)
// clang-format on

NAMESPACE_BEGIN

typedef pcl::PointCloud<FullPoint> FullPointCloud;
typedef pcl::PointCloud<velodyne_ros::Point> CloudVelodyne;
typedef pcl::PointCloud<ouster_ros::Point> CloudOuster;

NAMESPACE_END