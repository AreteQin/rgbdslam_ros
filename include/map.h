#ifndef MAP_H
#define MAP_H

#include "frame.h"
#include "common_include.h"
#include "mappoint.h"

namespace rgbd_slam {
    class Map {
    public:
        typedef std::unordered_map<unsigned long, std::shared_ptr<Frame>>
                KeyframesType;
        typedef std::unordered_map<unsigned long, std::shared_ptr<MapPoint>>
                LandmarksType;

        std::mutex map_mutex_;

        Map() {} // in order to make "new Map" available

        void InsertKeyframe(std::shared_ptr <Frame> frame);

        void InsertMapPoint(std::shared_ptr <MapPoint> map_point);

        KeyframesType GetAllKeyframes() {
            std::unique_lock <std::mutex> lock(map_mutex_);
            return keyframes_;
        }

        LandmarksType GetAllLandmarks() {
            std::unique_lock <std::mutex> lock(map_mutex_);
            return active_landmarks_;
        }

        KeyframesType GetActiveKeyframes() {
            std::unique_lock <std::mutex> lock(map_mutex_);
            return active_keyframes_;
        }

        LandmarksType GetActiveLandmarks() {
            std::unique_lock <std::mutex> lock(map_mutex_);
            return active_landmarks_;
        }

        // clean those points that are observed
        // less than "min_observed_times_"
        void CleanMap();

        // return the point cloud generated from all map points
        pcl::PointCloud <pcl::PointXYZRGB> GetPointCloud(Eigen::Matrix<double, 3, 3> K);

    private:
        void RemoveOldKeyframe();

        KeyframesType keyframes_; //store all keyframes
        LandmarksType landmarks_;
        KeyframesType active_keyframes_;
        LandmarksType active_landmarks_;
        std::shared_ptr <Frame> current_frame_ = nullptr;
        int num_of_active_keyframes_ = 10;
        int min_observed_times_ = 1;
    };
}
#endif