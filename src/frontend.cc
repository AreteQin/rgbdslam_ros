#include <opencv2/opencv.hpp>
#include "map.h"
#include "frontend.h"
#include "g2o_types.h"

namespace rgbd_slam {
    Frontend::Frontend() {}

    bool Frontend::AddFrame(std::shared_ptr<Frame> frame) {
        current_frame_ = frame;
        bool success = Track();
        return success;
    }

    bool Frontend::Track() {
        if (last_kf_) {
            current_frame_->SetPosition(relative_motion_ * last_kf_->Pose());
            LOG(INFO)<<"relative motion: \n"<<relative_motion_.matrix();
//            ExtractFeatures();
            num_generated_edges_ = EstimateCurrentPose();
            LOG(INFO) << "generated edges: " << num_generated_edges_;
            relative_motion_ =
                    current_frame_->Pose() * (last_kf_->Pose().inverse());
//            ref_frame_ = current_frame_;
        } else {
            InsertKeyframe(2);
            last_kf_ = current_frame_;
            ExtractFeatures();
            AddNewMapPoints();
            return true;
        }
        LOG(INFO) << "T_cur:\n" << current_frame_->Pose().matrix();
        double euclidean_distance = sqrt(
                0.5 * pow(relative_motion_.translation().x(), 2) +
                0.5 * pow(relative_motion_.translation().y(), 2) +
                0.5 * pow(relative_motion_.translation().z(), 2) +
                pow(relative_motion_.unit_quaternion().x(), 2) +
                pow(relative_motion_.unit_quaternion().y(), 2) +
                pow(relative_motion_.unit_quaternion().z(), 2) +
                pow(relative_motion_.unit_quaternion().w(), 2));
        LOG(INFO)<<"euclidean distance: "<<euclidean_distance;
        if (InsertKeyframe(euclidean_distance)) {
            last_kf_ = current_frame_;
            ExtractFeatures();
            AddNewMapPoints();
        }
        return true;
    }

    bool Frontend::InsertKeyframe(double euclidean_distance) {
        if (euclidean_distance < MAX_EUCLIDEAN_DISTANCE) {
            return false;
        }
        current_frame_->SetKeyframe();
        map_->InsertKeyframe(current_frame_);
        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;
        Eigen::Matrix3d R = Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0)).toRotationMatrix();
        Eigen::Vector3d t(0, 0, 0);
        relative_motion_ = Sophus::SE3d(R, t);
        //backend_->UpdateMap();
        return true;
    }

    int Frontend::ExtractFeatures() {
        for (int k = 0; k < last_kf_->color_image_.rows; k++) {
            for (int h = 0; h < last_kf_->color_image_.cols; h++) {
                double depth = last_kf_->depth_image_.at<unsigned short>(k, h);
                double color = last_kf_->color_image_.at<uchar>(k, h);

                if (depth == 0)
                    continue;
                last_kf_->depth_ref_.push_back(depth);
                last_kf_->color_ref_.push_back(color);

                last_kf_->features_.push_back(std::shared_ptr<Feature>(
                        // 1.0 is keypoint diameter
                        new Feature(last_kf_, cv::KeyPoint(h, k, 1.0),
                                    color)));
                last_kf_->num_good_points_++;
            }
        }
        // return last_kf_->depth_ref_.size(); it is working or not?
        LOG(INFO) << "Generated " << last_kf_->num_good_points_
                  << " 3D points in the last image.";
        return last_kf_->num_good_points_;
    }

    int Frontend::EstimateCurrentPose() {
        // 构建图优化，先设定g2o
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
        // 梯度下降方法，可以从GN, LM, DogLeg 中选
        auto solver = new g2o::OptimizationAlgorithmGaussNewton(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer; // 图模型
        optimizer.setAlgorithm(solver); // 设置求解器
        optimizer.setVerbose(false);    // 打开调试输出

        VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
        vertex_pose->setId(0);
        vertex_pose->setEstimate(relative_motion_);
        // vertex_pose->setMarginalized(true); // enable marginalization
        optimizer.addVertex(vertex_pose);

        // K
        Eigen::Matrix<double, 3, 3> K = camera_color_->K();

        // edges counter
        int edge_index = 1;

        for (size_t i = 0; i < last_kf_->features_.size(); ++i) {
            Eigen::Vector2d pixel_ref(
                    last_kf_->features_[i]->pixel_position_.pt.x,
                    last_kf_->features_[i]->pixel_position_.pt.y);
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(K,
                                                                      pixel_ref,
                                                                      last_kf_->color_ref_[i],
                                                                      last_kf_->depth_ref_[i],
                                                                      current_frame_->color_image_,
                                                                      current_frame_->depth_image_);

            edge->setId(edge_index);
            edge->setVertex(0, vertex_pose);

            // 深度乘以归一化坐标就得到了相机坐标系下的三维点
            Eigen::Vector3d position_in_ref_cam =
                    last_kf_->depth_ref_[i] * Eigen::Vector3d(
                            (pixel_ref[0] - K_()(0, 2)) / K_()(0, 0),
                            (pixel_ref[1] - K_()(1, 2)) / K_()(1, 1),
                            1);

            Eigen::Vector3d position_in_cur_cam =
                    vertex_pose->estimate() * position_in_ref_cam;
            Eigen::Matrix<double, 2, 1> measurements;
            measurements << last_kf_->color_ref_[i], position_in_cur_cam[2];
            edge->setMeasurement(measurements);
            edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
            optimizer.addEdge(edge);
            edge_index++;
        }
        LOG(INFO) << "start optimization";
        optimizer.initializeOptimization();
        optimizer.optimize(G2O_OPTIMIZATION_ITERATION);
        current_frame_->SetPosition(
                vertex_pose->estimate() * last_kf_->Pose());
        return edge_index;
    }

//    bool Frontend::BuildInitialMap() {
//        for (unsigned int i = 0; i < ref_frame_->num_good_points_; ++i) {
//            if (ref_frame_->features_[i] == nullptr)
//                continue;
//            // create map point from depth map
//            Eigen::Matrix<double, 3, 1> position_in_world =
//                    ref_frame_->depth_ref_[i]/DEPTH_SCALE * Eigen::Vector3d(
//                            (ref_frame_->features_[i]->pixel_position_.pt.x -
//                             K_()(0, 2)) / K_()(0, 0),
//                            (ref_frame_->features_[i]->pixel_position_.pt.y -
//                             K_()(1, 2)) / K_()(1, 1), 1);
//            if (position_in_world[2] > 0) {
//                auto new_map_point = MapPoint::CreateNewMappoint();
//                new_map_point->SetPosition(position_in_world);
//                ref_frame_->features_[i]->map_point_ = new_map_point;
//                map_->InsertMapPoint(new_map_point);
////                LOG(INFO) << "u,v,d: "
////                          << ref_frame_->features_[i]->pixel_position_.pt.x<<","
////                          << ref_frame_->features_[i]->pixel_position_.pt.y<<","
////                          << ref_frame_->depth_ref_[i];
////                LOG(INFO) << "x,y,z: \n" << position_in_world;
//            }
//        }
//        map_->InsertKeyframe(ref_frame_);
//        //backend_->UpdateMap();
//        LOG(INFO) << "Build initial map with "
//                  << current_frame_->num_good_points_ << " points.";
//        return true;
//    }

    bool Frontend::AddNewMapPoints() {
        Sophus::SE3 ref_Twc = last_kf_->Pose().inverse();
        for (unsigned int i = 0; i < last_kf_->num_good_points_; ++i) {
            if (last_kf_->features_[i] == nullptr)
                continue;
            // create map point from depth map
            Eigen::Vector3d position_in_camera =
                    last_kf_->depth_ref_[i] / DEPTH_SCALE * Eigen::Vector3d(
                            (last_kf_->features_[i]->pixel_position_.pt.x - K_()(0, 2)) / K_()(0, 0),
                            (last_kf_->features_[i]->pixel_position_.pt.y - K_()(1, 2)) / K_()(1, 1),
                            1);
            if (position_in_camera[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPosition(ref_Twc * position_in_camera);
                last_kf_->features_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
            }
        }
        LOG(INFO) << "Insert " << last_kf_->num_good_points_ << " map points";
        map_->InsertKeyframe(last_kf_);
        //backend_->UpdateMap();
        LOG(INFO) << "There are " << map_->GetAllLandmarks().size()
                  << " map points now";
        return true;
    }

}