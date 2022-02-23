#include "mappoint.h"

namespace rgbd_slam{
    std::shared_ptr<MapPoint> MapPoint::CreateNewMappoint(){
        static long factory_id = 0;
        std::shared_ptr<MapPoint> new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    }
}