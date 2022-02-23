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
    };

    void Frame::SetKeyframe()
    {
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    };

    std::shared_ptr<Frame> Frame::CreateFrame()
    {
        static long frame_factory_id = 0;
        std::shared_ptr<Frame> new_frame(new Frame);
        new_frame->id_ = frame_factory_id++;
        return new_frame;
    };
}