#include "map.h"
#include "feature.h"

namespace rgbd_slam {
    void Map::InsertKeyframe(std::shared_ptr<Frame> frame) {
        current_frame_ = frame;
//        // if the id of current frame is not in the map, then add it into the map, otherwise update the keyframes_
//        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) // .end() is throughout the unordered map
//        {
//            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
//            active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
//        }
//        else
//        {
//            keyframes_[frame->keyframe_id_] = frame;
//            active_keyframes_[frame->keyframe_id_] = frame;
//        }
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
//        if (active_keyframes_.size() > num_of_active_keyframes_)
//        {
//            RemoveOldKeyframe();
//        }
    }

    void Map::InsertMapPoint(std::shared_ptr<MapPoint> map_point) {
        if (landmarks_.find(map_point->id_) == landmarks_.end()) {
            landmarks_.insert(make_pair(map_point->id_, map_point));
            active_landmarks_.insert(make_pair(map_point->id_, map_point));
        } else {
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }
    }

    void Map::RemoveOldKeyframe() {
        if (current_frame_ == nullptr)
            return;
        double max_distance = 0, min_distance = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->Pose().inverse();
        // throughout all active frames, looking for the nearest frame and farthest frame to the current frame
        for (auto &kf: active_keyframes_) {
            if (kf.second ==
                current_frame_) // if kf points at current_frame_, do nothing, ".second" gets the value of iterator kf
                continue;
            auto distance = (kf.second->Pose() *
                             Twc).log().norm(); // get Lie Algebra format using log(), norm() represents 2 norm
            if (distance > max_distance) {
                max_distance = distance;
                max_kf_id = kf.first;
            }
            if (distance < min_distance) {
                min_distance = distance;
                min_kf_id = kf.first;
            }
        }
        const double min_distance_threshold = 0.2;
        std::shared_ptr<Frame> frame_to_remove = nullptr;
        // if the nearest frame is nearer than min_distance_threshold, remove it
        if (min_distance < min_distance_threshold) {
            frame_to_remove = keyframes_.at(min_kf_id);
        }
            // remove the farthest frame
        else {
            frame_to_remove = keyframes_.at(max_kf_id);
        }
        LOG(INFO) << "keyframe removed: " << frame_to_remove->keyframe_id_;
        // also remove it from the active keyframes
        active_keyframes_.erase(frame_to_remove->keyframe_id_);
        CleanMap();
    }

    void Map::CleanMap() {
        LOG(INFO) << "Map::CleanMap() undefined: " << min_observed_times_;
    }

    pcl::PointCloud<pcl::PointXYZRGB> Map::GetPointCloud(Eigen::Matrix<double, 3, 3> K) {

        std::unique_lock<std::mutex> lock(map_mutex_);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        point_cloud->header.frame_id = "map";
        pcl::PointXYZRGB p;

        // a rotation of Pi/2 degree along the x axis, and so on
        Eigen::Matrix3d R1 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
        Eigen::Matrix3d R2 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
        Eigen::Matrix3d R3 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        Eigen::Matrix3d R = R3*R2*R1;
        Eigen::Vector3d t(0, -10, 3);
        Sophus::SE3 T_wd(R,t);
        for (auto &landmark: active_landmarks_) {
            auto pointWorld = T_wd * landmark.second->Position();
            auto color = landmark.second->color_;
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
            p.b = color;
            p.g = color;
            p.r = color;
            point_cloud->points.push_back(p);
        }
        LOG(INFO) << "published " << point_cloud->size() << " points to ROS";
        return *point_cloud;
    }


}