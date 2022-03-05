#ifndef FRONTEND_H
#define FRONTEND_H

#include "frame.h"
#include "camera.h"
#include "common_include.h"
#include "map.h"
#include "backend.h"

namespace rgbd_slam {
    class Frontend {
    public:

        Frontend();

        bool AddFrame(std::shared_ptr<Frame> frame); // update the latest frame

        void LoadMap(std::shared_ptr<Map> map) {
            map_ = map;
        }

        void SetColorCamera(std::shared_ptr<Camera> camera_color) {
            camera_color_ = camera_color;
        }

        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

        Eigen::Matrix<double, 3, 3> K_() {
            return camera_color_->K();
        }

        std::shared_ptr<Frame> GetCurrentFrame(){
            return current_frame_;
        }

    private:
        // track in normal mode, return true if success
        bool Track();

        // generate pixels in ref and load depth data
        int ExtractFeatures();

        // estimate current frame's pose, return number of edges
        // (number of inliers)
        int EstimateCurrentPose();

        // insert current frame as a keyframe, return true if success
        bool InsertKeyframe(double euclidean_distance);

        // build the initial map with single frame, return true if success
        bool BuildInitialMap();

        // add new points into map generated from new keyframe, return true if success
        bool AddNewMapPoints();

        // number of inliers, used for determine new keyframe
        int num_generated_edges_ = 0;

        // threshold to determine new keyframe
        int num_edges_needed_for_keyframe_ = 2000;

        std::shared_ptr<Map> map_ = nullptr;
        std::shared_ptr<Frame> last_kf_ = nullptr;
        std::shared_ptr<Frame> current_frame_ = nullptr;
        std::shared_ptr<Camera> camera_color_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        Sophus::SE3d relative_motion_;
    };
}

#endif