#pragma once

#include "IncNdt.h"
#include "Viewer.h"

NAMESPACE_BEGIN

class IncNdtLo {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<IncNdtLo> Ptr;

    /// 用来描述跟踪状态
    enum class TrackState {
        NOT_INIT, ///< 还未初始化
        TRACKED,  ///< 跟踪正常
        LOST      ///< 跟踪丢失
    };

    /// Tracker的配置选项
    struct Options {
        double delta_position_; ///< 关键帧位置阈值
        double delta_angle_;    ///< 关键帧角度阈值
    };

    /// Tracker构造
    IncNdtLo(const Options &options);

    /// 添加点云帧，进行位姿估计
    void AddCloud(const PointCloud::Ptr &cloud, SE3d &Twl, bool use_guess = false);

    /// 设置可视化器
    void SetViewer(Viewer::Ptr viewer) { viewer_ = std::move(viewer); }

    /// 设置增量ndt配准器
    void SetIncNdt(IncNdt::Ptr inc_ndt) { inc_ndt_ = std::move(inc_ndt); }

private:
    /// 是否为关键帧
    bool IsKeyframe();

    IncNdt::Ptr inc_ndt_; ///< 增量ndt，维护局部地图
    Viewer::Ptr viewer_;  ///< 可视化器
    TrackState state_;    ///< 跟踪状态
    SE3d velocity_;       ///< 速度信息Tlc
    SE3d last_pose_;      ///< 上一次估计位姿
    SE3d curr_pose_;      ///< 当前位姿估计
    SE3d last_key_pose_;  ///< 上一关键帧位姿
    Options options_;     ///< Tracker配置项
    int frame_cnt_;       ///< 帧计数，判断是否添加新的关键帧
};

NAMESPACE_END