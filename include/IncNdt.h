#pragma once

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include "Common.hpp"

NAMESPACE_BEGIN

/**
 * @brief 增量NDT类
 * @details
 *      1. 维护体素内的特征数据，实现增量功能
 *      2. 进行NDT的3d点云配准，实现配装功能
 *      3. 实现活跃体素的前置更新，和旧体素的删除
 */
class IncNdt {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// 体素的搜索方式
    enum class NearbyType {
        CENTER, ///< 中心点
        NEARBY6 ///< 中心点+周围6个点
    };

    /// 体素数据结构
    struct Voxel3d {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// 更新体素正态分布属性
        void UpdateAttribute(const PointCloud::Ptr &cloud);

        /// 初始化体素正态分布属性
        void InitAttribute(const PointCloud::Ptr &cloud);
        
        void SetInfo();

        Mat3d cov_;  ///< 体素协方差矩阵
        Mat3d info_; ///< 体素的信息矩阵
        Vec3d mean_; ///< 体素的均值

        std::vector<int> buffer_;     ///< 点的缓冲区
        bool estimated_info_ = false; ///< 该体素是否已经估计了体素分布
        int last_buffer_size_ = -1;   ///< 上一次的buffer大小
    };

    /// IncNdt算法配置项
    struct Options {
        double voxel_size_;      ///< 体素尺寸
        double voxel_size_inv_;  ///< 体素尺寸的倒数
        int voxel_capacity_;     ///< 体素容量
        double out_lier_th_;     ///< NDT误差边阈值
        NearbyType nearby_type_; ///< 体素搜索方式
    };

    typedef std::shared_ptr<IncNdt> Ptr;
    typedef std::pair<Vec3i, Voxel3d> KeyAndVoxel;
    typedef std::list<KeyAndVoxel> VoxelList;
    typedef std::unordered_map<Vec3i, VoxelList::iterator, hash_vec3i> VoexlHashMap;

    IncNdt(Options options);

    /// 向体素中添加点云数据
    void AddCloud(PointCloud::Ptr cloud);

    /// 对点云进行配准
    int AlignG2O(PointCloud::Ptr cloud, SE3d &init_pose);

private:
    VoxelList voxels_;               ///< 所有的体素集合
    VoexlHashMap voxels_map_;        ///< 体素哈希表
    std::vector<Vec3i> nearby_keys_; ///< 周围体素
    Options options_;                ///< 增量ndt算法配置项
};

/**
 * @brief SE3顶点位姿
 *
 */
class SE3Vertex : public g2o::BaseVertex<6, SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool write(std::ostream &os) const override { return false; }

    bool read(std::istream &is) override { return false; }

    void setToOriginImpl() override { _estimate = SE3d(); }

    void oplusImpl(const double *update) override {
        _estimate.translation() += Vec3d(update[0], update[1], update[2]);
        _estimate.so3() = _estimate.so3() * Sophus::SO3d::exp(Vec3d(update[3], update[4], update[5]));
    }
};

/**
 * @brief NDT配准模型误差边
 *
 */
class NDTEdge : public g2o::BaseUnaryEdge<3, Vec3d, SE3Vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    NDTEdge(Vec3d voxel_mean)
        : mean_(voxel_mean) {}

    bool read(std::istream &is) override { return false; }

    bool write(std::ostream &os) const override { return false; }

    /// NDT的误差状态
    void computeError() override {
        SE3Vertex *v = dynamic_cast<SE3Vertex *>(_vertices[0]);
        const SE3d &Twb = v->estimate();
        _error = Twb * _measurement - mean_;
    }

    /// NDT的雅可比矩阵
    void linearizeOplus() override {
        SE3Vertex *v = dynamic_cast<SE3Vertex *>(_vertices[0]);
        const SE3d &Twb = v->estimate();
        _jacobianOplusXi.block<3, 3>(0, 0) = Mat3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3) = -Twb.so3().matrix() * Sophus::SO3d::hat(_measurement);
    }

private:
    Vec3d mean_; ///< 体素均值
};

NAMESPACE_END
