#ifndef BACKEND_H
#define BACKEND_H

#include "common_include.h"
#include "frame.h"
#include "map.h"
#include "camera.h"

namespace rgbd_slam
{
    class Backend
    {

    public:
        Backend();

        void SetCamera(std::shared_ptr<Camera> camera) { camera_ = camera; }

        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        void UpdateMap(); // update map and start optimization

        void Stop();

    private:
        void BackendLoop(); // backend thread

        void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex backend_mutex_;
        std::condition_variable map_update_;
        std::atomic<bool> backend_running_;
        std::shared_ptr<Camera> camera_;
    };
}

#endif // BACKEND_H