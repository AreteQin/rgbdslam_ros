#include "frame.h"

namespace rgbd_slam
{
    Frame::Frame(long id, double time_stamp, const Sophus::SE3d &pose,
                 const cv::Mat &color_image, const cv::Mat &depth_image)
    {
        id_ = id;
        time_stamp_ = time_stamp;
        pose_ = pose;
        color_image_ = color_image;
        depth_image_ = depth_image;
    }

    void Frame::SetKeyframe()
    {
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }

    std::shared_ptr<Frame> Frame::CreateFrame()
    {
        static long frame_factory_id = 0;
        std::shared_ptr<Frame> new_frame(new Frame);
        new_frame->id_ = frame_factory_id++;
        return new_frame;
    }

    pcl::PointCloud<pcl::PointXYZRGB> Frame::GetPointCloud() {

        std::unique_lock<std::mutex> lock(frame_mutex_);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        point_cloud->header.frame_id = "camera";
        pcl::PointXYZRGB p;

        for (auto &feature: features_) {
            //auto pointWorld = T_wd * landmark.second->Position();
            auto pointWorld = feature->map_point_->position_;
            auto color = feature->color_;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
            p.b = color;
            p.g = color;
            p.r = color;
            point_cloud->points.push_back(p);
        }
//        pcl::transformPointCloud(*point_cloud, *point_cloud, T_wd);
        LOG(INFO) << "published " << point_cloud->size() << " points to ROS";
        return *point_cloud;
    }
}