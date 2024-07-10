#pragma once

#include "IncNdt.h"
#include "Viewer.h"

NAMESPACE_BEGIN

class Tracker {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

        IncNdt::Options ndt_options_; ///< NDT的配置选项
    };

    /// Tracker构造
    Tracker(const Options &options);

    /// 添加点云帧，进行位姿估计
    void AddCloud(const PointCloud::Ptr &cloud, SE3d &Twl);

    /// 设置可视化器
    void SetViewer(Viewer::Ptr viewer) { viewer_ = std::move(viewer); }

private:
    /// 使用恒速模型跟踪
    void TrackMotionModel(const PointCloud::Ptr &cloud, SE3d &Twl);

    /// 是否为关键帧
    bool IsKeyframe(const SE3d &curr_pose);

    IncNdt::Ptr inc_ndt_; ///< 增量ndt，维护局部地图
    Viewer::Ptr viewer_;  ///< 可视化器
    TrackState state_;    ///< 跟踪状态
    SE3d velocity_;       ///< 速度信息Tlc
    SE3d last_pose_;      ///< 上一次估计位姿
    Options options_;     ///< Tracker配置项
    int frame_cnt_;       ///< 帧计数，判断是否添加新的关键帧
};

NAMESPACE_END