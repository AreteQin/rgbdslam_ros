#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"
#include "feature.h"

namespace rgbd_slam
{
    class Frame
    {
    public:
        unsigned long id_ = 0;
        bool is_keyframe_ = false;
        double time_stamp_;
        Sophus::SE3d pose_;
        std::mutex frame_mutex_;
        cv::Mat color_image_, depth_image_;
        unsigned long keyframe_id_ = 0;
        uint32_t num_good_points_ = 0;
        std::vector<std::shared_ptr<Feature>> features_;
        std::vector<double> color_ref_, depth_ref_;

        Frame() {} // in order to make "new Frame" available
        Frame(long id, double time_stamp, const Sophus::SE3d &pose,
              const cv::Mat &color_image, const cv::Mat &depth_image);
        Sophus::SE3d Pose()
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            return pose_;
        }
        void SetPosition(const Sophus::SE3d &pose)
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            pose_ = pose;
        }
        void SetKeyframe();
        static std::shared_ptr<Frame> CreateFrame();
    };
}
#endif