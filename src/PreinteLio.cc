#include <execution>

#include <g2o/core/robust_kernel_impl.h>
#include <pcl/common/transforms.h>

#include "CloudConvert.h"
#include "G2oTypes.h"
#include "PreinteLio.h"

NAMESPACE_BEGIN

/**
 * @brief 预积分耦合系统添加点云信息
 *
 * @param full_cloud        输入的全量点云
 * @param laser_start_time  输入的全量点云的开始时间戳
 */
void PreinteLio::AddCloud(const FullPointCloud::Ptr &full_cloud, double laser_start_time) {
    if (!imu_static_init_->init_success_)
        return;

    if (first_laser_) {
        first_laser_ = false;

        // clang-format off
        IMUPreintegration::Options preinte_options = {
            options_.remove_gravity_,    
            imu_static_init_->acc_bias_,
            imu_static_init_->gyr_bias_,
            imu_static_init_->gravity_,
            imu_static_init_->sigma_a_,  
            imu_static_init_->sigma_g_
        };
        // clang-format on

        PointCloud::Ptr laser_cloud = CloudConvert::Full2PointCloud(full_cloud);
        laser_cloud = VoxelCloud(laser_cloud, 0.1);
        PointCloud::Ptr imu_cloud = pcl::make_shared<PointCloud>();
        pcl::transformPointCloud(*laser_cloud, *imu_cloud, options_.Tli_.inverse().matrix().cast<float>());
        inc_ndt_->AddCloud(imu_cloud);

        double laser_end_time = laser_start_time + full_cloud->back().time;
        preint_ = std::make_shared<IMUPreintegration>(preinte_options);
        preint_->SetLaserStamp(laser_end_time);

        state_timei_.Twi_ = last_key_Twi_ = SE3d();
        state_timei_.stamp_ = laser_end_time;
        state_timei_.ba_ = imu_static_init_->acc_bias_;
        state_timei_.bg_ = imu_static_init_->gyr_bias_;
        state_timei_.g_ = imu_static_init_->gravity_;
        state_timei_.v_ = Vec3d::Zero();

        if (viewer_)
            viewer_->SetPoseAndCloud(state_timei_.Twi_, imu_cloud);
    };

    cloud_buffer_.push_back(full_cloud);
    start_time_buffer_.push_back(laser_start_time);
    end_time_buffer_.push_back(laser_start_time + full_cloud->back().time);
}

/**
 * @brief 预积分耦合系统添加imu数据
 *
 * @param imu 输入的imu数据
 */
void PreinteLio::AddIMU(const IMU::Ptr &imu) {
    if (!imu_static_init_->init_success_) {
        imu_static_init_->AddIMU(imu);
        return;
    }

    if (first_laser_)
        return;

    if ((cloud_buffer_.size() == 1 && imu->stamp_ > end_time_buffer_.front()) ||
        (cloud_buffer_.size() > 1 && imu->stamp_ > end_time_buffer_.front() && imu->stamp_ < end_time_buffer_[1])) {
        preint_->Integrate(imu, imu->stamp_);
        state_timej_ = preint_->Predict(state_timei_);
        Twi_query_.push_back(state_timej_.Twi_);
        stamp_query_.push_back(imu->stamp_);
    } else if (cloud_buffer_.size() > 1 && imu->stamp_ > end_time_buffer_[1]) {
        preint_->Integrate(imu, end_time_buffer_[1]);
        state_timej_ = preint_->Predict(state_timei_);

        PointCloud::Ptr laser_cloud = UndistortCloud(cloud_buffer_[1], start_time_buffer_[1]);
        laser_cloud = VoxelCloud(laser_cloud, 0.1);
        auto imu_cloud = pcl::make_shared<PointCloud>();
        pcl::transformPointCloud(*laser_cloud, *imu_cloud, options_.Tli_.inverse().matrix().cast<float>());

        SE3d init_pose = state_timej_.Twi_;
        inc_ndt_->AlignG2O(imu_cloud, init_pose);
        Optimize(init_pose);

        preint_->Reset(state_timej_.bg_, state_timej_.ba_);
        NormalizeVelocity();
        state_timei_ = state_timej_;

        auto world_cloud = pcl::make_shared<PointCloud>();
        if (viewer_) {
            pcl::transformPointCloud(*imu_cloud, *world_cloud, state_timej_.Twi_.matrix().cast<float>());
            viewer_->SetPoseAndCloud(state_timej_.Twi_, world_cloud);
        }
        ++frame_cnt_;
        if (IsKeyframe()) {
            // RCLCPP_INFO(rclcpp::get_logger("inc_ndt_lins"), "添加关键帧");
            frame_cnt_ = 0;
            last_key_Twi_ = state_timej_.Twi_;
            if (!viewer_)
                pcl::transformPointCloud(*imu_cloud, *world_cloud, state_timej_.Twi_.matrix().cast<float>());
            inc_ndt_->AddCloud(world_cloud);
        }

        start_time_buffer_.pop_front();
        end_time_buffer_.pop_front();
        cloud_buffer_.pop_front();
    }
}

/**
 * @brief 使用位姿差值进行点云的运动矫正
 *
 * @return PointCloud::Ptr 输出的矫正后的普通点云信息，用于后续的配准工作
 */
PointCloud::Ptr PreinteLio::UndistortCloud(const FullPointCloud::Ptr &full_cloud, const double &laser_start_time) {
    SE3d Tlw_ts = options_.Tli_ * state_timej_.Twi_.inverse();
    std::vector<int> indices(full_cloud->size());
    std::for_each(indices.begin(), indices.end(), [idx = 0](int &id) mutable { id = idx++; });

    std::vector<PointT> points_out(full_cloud->size());
    std::vector<bool> points_valid(full_cloud->size(), true);
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
        SE3d Twl_t;
        const auto &pt_in = full_cloud->points[idx];
        if (!DifferencePose(laser_start_time + pt_in.time, Twl_t)) {
            points_valid[idx] = false;
            return;
        }
        Vec3d pt_ts = Tlw_ts * Twl_t * to_vec3d(pt_in);
        PointT pt_out;
        pt_out.x = pt_ts[0];
        pt_out.y = pt_ts[1];
        pt_out.z = pt_ts[2];
        points_out[idx] = pt_out;
    });

    PointCloud::Ptr pcl_out = pcl::make_shared<PointCloud>();
    std::for_each(indices.begin(), indices.end(), [&](const int &idx) {
        if (points_valid[idx]) {
            pcl_out->push_back(points_out[idx]);
        }
    });

    /// 重新组织位姿差值查询数据库
    std::vector<double> stamp_query;
    std::vector<SE3d> Twi_query;

    std::swap(stamp_query, stamp_query_);
    std::swap(Twi_query, Twi_query_);

    return pcl_out;
}

/// 对位姿进行差值
bool PreinteLio::DifferencePose(const double &query_stamp, SE3d &Twl_t) {
    const double &earliest_time = stamp_query_.front();
    const double &latest_time = stamp_query_.back();

    if (query_stamp <= earliest_time) {
        /// 位姿差值超出时间范围50ms
        if (earliest_time - query_stamp > 0.05)
            return false;
        Twl_t = Twi_query_.front() * options_.Tli_.inverse();
        return true;
    }

    if (query_stamp >= latest_time) {
        if (query_stamp - latest_time > 0.05)
            return false;
        Twl_t = Twi_query_.back() * options_.Tli_.inverse();
        return true;
    }

    for (int i = 0; i < stamp_query_.size() - 1; ++i) {
        const double &stamp_i = stamp_query_[i];
        const double &stamp_j = stamp_query_[i + 1];
        if (stamp_i < query_stamp && stamp_j >= query_stamp) {
            double dt = stamp_j - stamp_i;
            if (dt < 1e-4) {
                Twl_t = Twi_query_[i] * options_.Tli_.inverse();
                break;
            }
            double s = (query_stamp - stamp_i) / dt;
            Vec3d t_interp = s * Twi_query_[i].translation() + (1 - s) * Twi_query_[i + 1].translation();
            Eigen::Quaterniond R_interp = Twi_query_[i].unit_quaternion().slerp(s, Twi_query_[i + 1].unit_quaternion());
            Twl_t = SE3d(R_interp, t_interp) * options_.Tli_.inverse();
            break;
        }
    }
    return true;
}

/**
 * @brief 进行整体性优化
 *
 */
void PreinteLio::Optimize(const SE3d &ndt_pose) {
    Optimizer optimizer;
    auto lm = new Levenberg(std::make_unique<BlockSolver>(std::make_unique<LinearSolver>()));
    optimizer.setAlgorithm(lm);

    int vertex_id = 0, edge_id = 0;

    /// 创建顶点
    auto vTi = new SE3Vertex;
    vTi->setId(vertex_id++);
    vTi->setEstimate(state_timei_.Twi_);
    vTi->setFixed(false);
    optimizer.addVertex(vTi);

    auto vTj = new SE3Vertex;
    vTj->setId(vertex_id++);
    vTj->setEstimate(ndt_pose);
    optimizer.addVertex(vTj);

    auto vvi = new VelocityVertex;
    vvi->setId(vertex_id++);
    vvi->setEstimate(state_timei_.v_);
    optimizer.addVertex(vvi);

    auto vvj = new VelocityVertex;
    vvj->setId(vertex_id++);
    vvj->setEstimate(state_timej_.v_);
    optimizer.addVertex(vvj);

    auto vbgi = new GyrBiasVertex;
    vbgi->setId(vertex_id++);
    vbgi->setEstimate(state_timei_.bg_);
    vbgi->setFixed(true);
    optimizer.addVertex(vbgi);

    auto vbgj = new GyrBiasVertex;
    vbgj->setId(vertex_id++);
    vbgj->setEstimate(state_timej_.bg_);
    optimizer.addVertex(vbgj);

    auto vbai = new AccBiasVertex;
    vbai->setId(vertex_id++);
    vbai->setEstimate(state_timei_.ba_);
    vbai->setFixed(true);
    optimizer.addVertex(vbai);

    auto vbaj = new AccBiasVertex;
    vbaj->setId(vertex_id++);
    vbaj->setEstimate(state_timej_.ba_);
    optimizer.addVertex(vbaj);

    /// 创建优化边
    IntegEdge *integ_edge = new IntegEdge(preint_);
    integ_edge->setId(edge_id++);
    integ_edge->setVertex(0, vTi);
    integ_edge->setVertex(1, vvi);
    integ_edge->setVertex(2, vbgi);
    integ_edge->setVertex(3, vbai);
    integ_edge->setVertex(4, vTj);
    integ_edge->setVertex(5, vvj);
    auto *rk = new g2o::RobustKernelHuber();
    rk->setDelta(200.0);
    integ_edge->setRobustKernel(rk);
    integ_edge->computeError();
    optimizer.addEdge(integ_edge);

    GyrBiasEdge *gyr_bias = new GyrBiasEdge;
    gyr_bias->setId(edge_id++);
    gyr_bias->setInformation(Mat3d::Identity() * 1e4);
    gyr_bias->setVertex(0, vbgi);
    gyr_bias->setVertex(1, vbgj);
    optimizer.addEdge(gyr_bias);

    AccBiasEdge *acc_bias = new AccBiasEdge;
    acc_bias->setId(edge_id++);
    acc_bias->setInformation(Mat3d::Identity() * 1e4);
    acc_bias->setVertex(0, vbai);
    acc_bias->setVertex(1, vbaj);
    optimizer.addEdge(acc_bias);

    PriorEdge *prior_edge = new PriorEdge(state_timei_);
    prior_edge->setId(edge_id++);
    prior_edge->setInformation(prior_info_);
    prior_edge->setVertex(0, vTi);
    prior_edge->setVertex(1, vvi);
    prior_edge->setVertex(2, vbgi);
    prior_edge->setVertex(3, vbai);
    prior_edge->computeError();
    optimizer.addEdge(prior_edge);

    NdtObserveEdge *ndt_edge = new NdtObserveEdge;
    ndt_edge->setId(edge_id++);
    ndt_edge->setInformation(ndt_info_);
    ndt_edge->setVertex(0, vTj);
    ndt_edge->setMeasurement(ndt_pose);
    ndt_edge->computeError();
    optimizer.addEdge(ndt_edge);

    optimizer.setVerbose(false);
    integ_edge->computeError();
    optimizer.initializeOptimization();
    optimizer.optimize(15);

    /// 将优化完成的结果放到state_time_i和state_time_j中
    state_timei_.Twi_ = vTi->estimate();
    state_timei_.v_ = vvi->estimate();
    state_timei_.ba_ = vbai->estimate();
    state_timei_.bg_ = vbgi->estimate();

    state_timej_.Twi_ = vTj->estimate();
    state_timej_.v_ = vvj->estimate();
    state_timej_.ba_ = vbaj->estimate();
    state_timej_.bg_ = vbgj->estimate();

    /// 重置先验信息矩阵
    Mat30d H = Mat30d::Zero();
    H.block<24, 24>(0, 0) += integ_edge->GetHessian();
    Mat6d Hgr = gyr_bias->GetHessian();
    Mat6d Har = acc_bias->GetHessian();
    H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
    H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);
    H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
    H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
    H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);
    H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
    H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
    H.block<15, 15>(0, 0) += prior_edge->GetHessian();
    H.block<6, 6>(15, 15) += ndt_edge->GetHessian();
    H = Marginalize(H, 0, 14);
    prior_info_ = H.block<15, 15>(15, 15);

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("inc_ndt_lins"),
    //                    state_timej_.Twi_.translation().transpose()
    //                        << "\t" << state_timej_.Twi_.so3().unit_quaternion().coeffs().transpose());
}

/**
 * @brief 判断LIO系统是否需要向inc_ndt中添加点云数据
 *
 * @return true     需要添加
 * @return false    不需要添加
 */
bool PreinteLio::IsKeyframe() {
    Vec3d delta_t = state_timej_.Twi_.translation() - last_key_Twi_.translation();
    if (delta_t.norm() > options_.delta_position_)
        return true;

    Vec3d delta_r = (state_timej_.Twi_.so3() * last_key_Twi_.so3().inverse()).log();
    if (delta_r.norm() > options_.delta_angle_)
        return true;

    return false;
}

/**
 * @brief 标准差，不太清楚这么做的目的
 *
 */
void PreinteLio::NormalizeVelocity() {
    Vec3d v_body = state_timej_.Twi_.so3().inverse() * state_timej_.v_;
    if (v_body[1] > 0)
        v_body[1] = 0;
    v_body[2] = 0;
    if (v_body[1] < -2.0)
        v_body[1] = -2.0;

    if (v_body[0] > 0.1) {
        v_body[0] = 0.1;
    } else if (v_body[0] < -0.1) {
        v_body[0] = -0.1;
    }

    state_timej_.v_ = state_timej_.Twi_.so3() * v_body;
}

NAMESPACE_END
