#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"
#include "frame.h"

namespace rgbd_slam
{
    class MapPoint
    {
    public:
        unsigned long id_ = 0;
        bool is_outlier_ = false;
        Eigen::Matrix<double, 3, 1> position_ = Eigen::Matrix<double, 3, 1>::Zero();
        double color_;
        std::mutex mappoint_mutex_;
        int observed_times_ = 0;

        MapPoint() {}

        MapPoint(long id, Eigen::Matrix<double, 3, 1> position, double color):id_(id), position_(position), color_(color) {}

//        Eigen::Matrix<double, 3, 1> Position()
//        {
//            std::unique_lock<std::mutex> lock(mappoint_mutex_);
//            return position_;
//        }

        void SetPosition(const Eigen::Matrix<double, 3, 1> &position)
        {
            std::unique_lock<std::mutex> lock(mappoint_mutex_);
            position_ = position;
        }

        static std::shared_ptr<MapPoint> CreateNewMappoint();
    };
}

#endif