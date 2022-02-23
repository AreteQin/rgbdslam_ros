#include <opencv2/opencv.hpp>
#include "backend.h"
#include "feature.h"
#include "map.h"
#include "mappoint.h"

namespace rgbd_slam {

    Backend::Backend(){
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    void Backend::UpdateMap(){
        std::unique_lock<std::mutex> lock(backend_mutex_);
        map_update_.notify_one(); // notify a waiting thread that the data can be read
    }

    void Backend::Stop(){
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop(){
        while (backend_running_.load()){
            std::unique_lock<std::mutex> lock(backend_mutex_);
            map_update_.wait(lock);
            Map::KeyframesType active_keyframes = map_->GetActiveKeyframes();
            Map::LandmarksType active_landmarks = map_->GetActiveLandmarks();
            // Optimize(active_keyframes, active_landmarks);
        }
    }

}