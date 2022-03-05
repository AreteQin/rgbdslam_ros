#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "common_include.h"
#include "frontend.h"
#include "map.h"

namespace rgbd_slam {
    class VisualOdometry {
    public:
        VisualOdometry(std::shared_ptr <Camera> camera_color);

        bool Initialize(); // return true if succeeded

        bool AddFrame(std::shared_ptr <Frame> frame);

        std::shared_ptr <Map> GetMap() {
            return map_;
        }

        pcl::PointCloud <pcl::PointXYZRGB> GetPointCloud(){
//            return map_->GetPointCloud(camera_color_->K());
            return frontend_->GetCurrentFrame()->GetPointCloud();
        }

        Sophus::SE3d GetCurrentFramePose(){
            return frontend_->GetCurrentFrame()->Pose();
        }

    private:
        std::shared_ptr <Camera> camera_color_;
        bool initialized_ = false;
        std::shared_ptr <Frontend> frontend_ = nullptr;
        std::shared_ptr <Map> map_ = nullptr;
        std::shared_ptr <Backend> backend_ = nullptr;
    };
}
#endif