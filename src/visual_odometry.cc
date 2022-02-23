#include "visual_odometry.h"

namespace rgbd_slam {

    VisualOdometry::VisualOdometry(std::shared_ptr <Camera> camera_color)
            : camera_color_(camera_color) {}

    bool VisualOdometry::Initialize() {
        frontend_ = std::shared_ptr<Frontend>(new Frontend);
        map_ = std::shared_ptr<Map>(new Map);
        backend_ = std::shared_ptr<Backend>(new Backend);

        frontend_->SetColorCamera(camera_color_);
        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        LOG(INFO) << "frontend initialized";
        backend_->SetMap(map_);
        LOG(INFO) << "map initialized";
        backend_->SetCamera(camera_color_);
        LOG(INFO) << "beckend initialized";
        return true;
    }

    bool VisualOdometry::AddFrame(std::shared_ptr <Frame> frame) {
        std::shared_ptr <Frame> new_frame = frame;
        if (new_frame == nullptr)
            return false;
        LOG(INFO) << "VO is running -------------------------------------------";
        bool success = frontend_->AddFrame(new_frame);
        return success;
    }

}