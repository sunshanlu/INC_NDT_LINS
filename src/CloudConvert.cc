#include <execution>

#include <pcl_conversions/pcl_conversions.h>

#include "CloudConvert.hpp"

NAMESPACE_BEGIN

/**
 * @brief velodyne多线激光雷达点云数据转换函数
 *
 * @param cloud_msg 输入的原始点云消息数据
 * @param pcl_out   输出的转化后的全量点云数据
 */
void CloudConvert::VelodyneHandler(const PointCloudPtr &cloud_msg, FullPointCloud::Ptr &pcl_out) {
    const static float omega = 3.61; // °/ms

    auto cloud_in = pcl::make_shared<CloudVelodyne>();
    auto cloud_out = pcl::make_shared<FullPointCloud>();

    pcl::fromROSMsg(*cloud_msg, *cloud_in);
    cloud_out->points.reserve(cloud_in->size());

    std::vector<bool> init_start_angle(options_.num_scans_, false);
    std::vector<float> start_angle(options_.num_scans_, 0.0);

    for (int i = 0; i < cloud_in->size(); ++i) {
        const auto &pt = cloud_in->points[i];
        const auto &ring = pt.ring;
        if (!init_start_angle[ring]) {
            float angle = std::atan2(pt.y, pt.x);
            start_angle[ring] = angle;
            init_start_angle[ring] = true;
        }
    }

    std::vector<int> indices(cloud_in->points.size());
    std::vector<bool> transformed(cloud_in->points.size(), false);
    std::vector<FullPoint> full_points(cloud_in->points.size());

    std::for_each(indices.begin(), indices.end(), [id = 0](int &idx) mutable { idx = id++; });
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
        if (idx % options_.point_filter_ != 0)
            return;
        const auto &pt = cloud_in->points[idx];
        const auto &ring = pt.ring;
        FullPoint fpt;
        float pt_angle = std::atan2(pt.y, pt.x);
        if (pt_angle <= start_angle[ring]) {
            float delta_angle = (start_angle[ring] - pt_angle) * 180.0 / M_PI;
            fpt.time = delta_angle / omega;
        } else {
            float delta_angle = (start_angle[ring] - pt_angle + 2 * M_PI) * 180.0 / M_PI;
            fpt.time = delta_angle / omega;
        }
        fpt.x = pt.x;
        fpt.y = pt.y;
        fpt.z = pt.z;
        fpt.ring = pt.ring;
        fpt.intensity = pt.intensity;
        full_points[idx] = fpt;
        transformed[idx] = true;
    });

    std::for_each(indices.begin(), indices.end(), [&](const int &idx) {
        if (transformed[idx])
            cloud_out->points.push_back(full_points[idx]);
    });
    cloud_out->points.resize(cloud_out->points.size());

    pcl_out = cloud_out;
}

/**
 * @brief Ouster 激光雷达点云数据转换函数
 *
 * @param cloud_msg 输入的原始点云消息数据
 * @param pcl_out   输出的转化后的全量点云数据
 */
void CloudConvert::OustHandler(const PointCloudPtr &cloud_msg, FullPointCloud::Ptr &pcl_out) {
    auto cloud_in = pcl::make_shared<CloudOuster>();
    auto cloud_out = pcl::make_shared<FullPointCloud>();
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    std::vector<int> need_indices;
    for (int i = 0; i < cloud_in->size(); ++i) {
        if (i % options_.point_filter_ == 0)
            need_indices.push_back(i);
    }

    std::vector<int> indices(need_indices.size());
    std::for_each(indices.begin(), indices.end(), [id = 0](int &idx) mutable { idx = id++; });
    cloud_out->points.resize(need_indices.size());
    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
        const int &need_id = need_indices[idx];
        const auto &pt = cloud_in->points[need_id];
        FullPoint fpt;
        fpt.x = pt.x;
        fpt.y = pt.y;
        fpt.z = pt.z;
        fpt.intensity = pt.intensity;
        fpt.ring = pt.ring;
        fpt.time = pt.t / 1e6;
        cloud_out->points[idx] = fpt;
    });
    pcl_out = cloud_out;
}

/**
 * @brief 大疆激光雷达点云转换处理逻辑
 *
 * @param cloud_msg 输入的原始点云消息数据
 * @param pcl_out   输出的转化后的全量点云数据
 */
void CloudConvert::LivoxHandler(const LivoxCloud::SharedPtr &cloud_msg, FullPointCloud::Ptr &pcl_out) {
    int plsize = cloud_msg->point_num;
    pcl_out = pcl::make_shared<FullPointCloud>();
    pcl_out->points.reserve(plsize);

    std::vector<int> need_indices;
    for (int i = 0; i < plsize; ++i) {
        if (i % options_.point_filter_ == 0)
            need_indices.push_back(i);
    }
    std::vector<bool> is_valid_pt(need_indices.size(), false);
    std::vector<FullPoint> full_points(need_indices.size());
    std::vector<int> indices(need_indices.size() - 1);
    std::for_each(indices.begin(), indices.end(), [id = 1](int &idx) mutable { idx = id++; });

    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const int &idx) {
        int need_id = need_indices[idx];
        int last_need_id = need_indices[idx - 1];
        const auto &need_pt = cloud_msg->points[need_id];
        const auto &last_need_pt = cloud_msg->points[last_need_id];
        int ret = need_pt.tag & 0x30;
        if (need_pt.line < options_.num_scans_ && (ret == 0x10 || ret == 0x00)) {
            if (std::fabs(last_need_pt.x - need_pt.x) < 1e-7 && std::fabs(last_need_pt.y - need_pt.y) < 1e-7 &&
                std::fabs(last_need_pt.z - need_pt.z) < 1e-7)
                return;
            FullPoint fpt;
            fpt.x = need_pt.x;
            fpt.y = need_pt.y;
            fpt.z = need_pt.z;
            fpt.intensity = need_pt.reflectivity;
            fpt.time = need_pt.offset_time * 1e-6;
            is_valid_pt[idx] = true;
            full_points[idx] = fpt;
        }
    });

    std::for_each(indices.begin(), indices.end(), [&](const int &idx) {
        if (is_valid_pt[idx])
            pcl_out->points.push_back(full_points[idx]);
    });
    pcl_out->points.resize(pcl_out->points.size());
}

NAMESPACE_END