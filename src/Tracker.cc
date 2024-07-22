#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>

#include "Tracker.h"

NAMESPACE_BEGIN

Tracker::Tracker(const Options &options)
    : state_(TrackState::NOT_INIT)
    , options_(options)
    , frame_cnt_(0)
    , track_motion_(true) {
    inc_ndt_ = std::make_shared<IncNdt>(options.ndt_options_);
}

void Tracker::AddCloud(const PointCloud::Ptr &cloud, SE3d &Twl) {
    switch (state_) {
    case TrackState::NOT_INIT:
        inc_ndt_->AddCloud(cloud);
        state_ = TrackState::TRACKED;
        last_pose_ = last_key_pose_ = Twl = SE3d();
        break;

    case TrackState::TRACKED:
        TrackMotionModel(cloud, Twl);
        break;

    default:
        throw std::runtime_error("跟踪丢失！");
    }
}

/**
 * @brief 使用速度进行位姿预测
 *
 * @param cloud 输入的待配准的点云
 * @param Twl   输出的配准位姿
 */
void Tracker::TrackMotionModel(const PointCloud::Ptr &cloud, SE3d &Twl) {
    if (track_motion_)
        curr_pose_ = last_pose_ * velocity_;
    int ninlier = inc_ndt_->AlignG2O(cloud, curr_pose_);
    ++frame_cnt_;
    if (ninlier < 10) {
        RCLCPP_ERROR(rclcpp::get_logger("inc_ndt_lins"), "跟踪比例为: %d", ninlier);
        // state_ = TrackState::LOST;
        // return;
    }

    PointCloud::Ptr cloud_world = pcl::make_shared<PointCloud>();
    if (viewer_) {
        pcl::transformPointCloud(*cloud, *cloud_world, curr_pose_.matrix());
        viewer_->SetPoseAndCloud(curr_pose_, cloud_world);
    }

    if (IsKeyframe(curr_pose_)) {
        frame_cnt_ = 0;
        last_key_pose_ = curr_pose_;
        if (!viewer_)
            pcl::transformPointCloud(*cloud, *cloud_world, curr_pose_.matrix());
        inc_ndt_->AddCloud(cloud_world);
    }

    Twl = curr_pose_;
    velocity_ = last_pose_.inverse() * curr_pose_;
    last_pose_ = curr_pose_;
}

/**
 * @brief 是否为关键帧
 *
 * @param curr_pose 当前帧的估计位姿
 * @return true     是关键帧
 * @return false    不是关键帧
 */
bool Tracker::IsKeyframe(const SE3d &curr_pose) {
    if (frame_cnt_ > 10)
        return true;

    Vec3d delta_t = curr_pose.translation() - last_key_pose_.translation();
    double delta_position = delta_t.lpNorm<2>();
    if (delta_position > options_.delta_position_)
        return true;

    Vec3d delta_r = (last_key_pose_.inverse() * curr_pose).so3().log();
    double delta_angle = delta_r.lpNorm<2>();
    if (delta_angle > options_.delta_angle_)
        return true;
    return false;
}

NAMESPACE_END