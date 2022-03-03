#ifndef FEATURE_H
#define FEATURE_H

#include "common_include.h"
// #include "mappoint.h"
// #include "frame.h" // why is it not passed?

namespace rgbd_slam
{
    class Frame;
    class MapPoint;
    class Feature
    {
    public:
        std::weak_ptr<Frame> frame_;        // frame that contains this feature
        cv::KeyPoint pixel_position_;       // 2D position in frame_
        double color_;
        std::shared_ptr<MapPoint> map_point_; // coresponding 3D map point
        Feature() {}
        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &pixel_position, double color)
            : frame_(frame), pixel_position_(pixel_position), color_(color) {}
    };
}

#endif // FEATURE_H