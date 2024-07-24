#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>

#include "IncNdtLo.h"

NAMESPACE_BEGIN

IncNdtLo::IncNdtLo(const Options &options)
    : state_(TrackState::NOT_INIT)
    , options_(options)
    , frame_cnt_(0) {}

void IncNdtLo::AddCloud(const PointCloud::Ptr &cloud, SE3d &Twl, bool use_guess) {
    if (state_ == TrackState::NOT_INIT) {
        inc_ndt_->AddCloud(cloud);
        state_ = TrackState::TRACKED;
        last_pose_ = last_key_pose_ = Twl = SE3d();
        return;
    }

    SE3d estimate_pose = curr_pose_;
    if (!use_guess)
        estimate_pose = last_pose_ * velocity_;
    else
        estimate_pose = Twl;

    inc_ndt_->AlignG2O(cloud, estimate_pose);
    ++frame_cnt_;
    curr_pose_ = estimate_pose;

    PointCloud::Ptr cloud_world = pcl::make_shared<PointCloud>();
    if (viewer_) {
        pcl::transformPointCloud(*cloud, *cloud_world, curr_pose_.matrix());
        viewer_->SetPoseAndCloud(curr_pose_, cloud_world);
    }

    if (IsKeyframe()) {
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
bool IncNdtLo::IsKeyframe() {
    if (frame_cnt_ > 10)
        return true;

    Vec3d delta_t = curr_pose_.translation() - last_key_pose_.translation();
    double delta_position = delta_t.lpNorm<2>();
    if (delta_position > options_.delta_position_)
        return true;

    Vec3d delta_r = (last_key_pose_.inverse() * curr_pose_).so3().log();
    double delta_angle = delta_r.lpNorm<2>();
    if (delta_angle > options_.delta_angle_)
        return true;
    return false;
}

NAMESPACE_END