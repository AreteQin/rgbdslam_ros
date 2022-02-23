#include <opencv2/opencv.hpp>
#include "map.h"
#include "frontend.h"
#include "g2o_types.h"

namespace rgbd_slam {
    Frontend::Frontend() {}

    bool Frontend::AddFrame(std::shared_ptr <Frame> frame) {
        current_frame_ = frame;
        bool success = Track();
        return success;
    }

    bool Frontend::Track() {
        if (ref_frame_) {
            current_frame_->SetPosition(relative_motion_ * ref_frame_->Pose());
            ExtractFeatures();
            num_generated_edges_ = EstimateCurrentPose();
            LOG(INFO) << "generated edges: " << num_generated_edges_;
            relative_motion_ =
                    current_frame_->Pose() * (ref_frame_->Pose().inverse());
            AddNewMapPoints();
            ref_frame_ = current_frame_;
        } else {
            ref_frame_ = current_frame_;
            ExtractFeatures();
            BuildInitialMap();
        }
        LOG(INFO)<<"T_cur:\n"<<current_frame_->Pose().matrix();
        InsertKeyframe();
        return true;
    }

    bool Frontend::InsertKeyframe() {
        // if (num_generated_edges_ >= num_edges_needed_for_keyframe_)
        // {
        //     return false; // still have enough points
        // }
        current_frame_->SetKeyframe();
        map_->InsertKeyframe(current_frame_);
        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;
        //backend_->UpdateMap();
        return true;
    }

    int Frontend::ExtractFeatures()
    {
        for (int k = 0; k < ref_frame_->color_image_.rows; k++) {
            for (int h = 0; h < ref_frame_->color_image_.cols; h++) {
                double depth = ref_frame_->depth_image_.at<unsigned short>(k,h)/DEPTH_SCALE;
                LOG(INFO)<<"depth: "<<depth;
                double color = ref_frame_->color_image_.at<uchar>(k, h);
                if (depth < MIN_DEPTH || depth > MAX_DEPTH)
                    continue;
                ref_frame_->depth_ref_.push_back(depth);
                ref_frame_->color_ref_.push_back(color);

                ref_frame_->features_.push_back(std::shared_ptr<Feature>(
                        // 1.0 is keypoint diameter
                        new Feature(ref_frame_, cv::KeyPoint(h, k, 1.0),
                                    color)));
                ref_frame_->num_good_points_++;
            }
        }
        // return ref_frame_->depth_ref_.size(); it is working or not?
        LOG(INFO) << "Generated " << ref_frame_->num_good_points_
                  << " 3D points in the last image.";
        return ref_frame_->num_good_points_;
    }

    int Frontend::EstimateCurrentPose() {
        // 构建图优化，先设定g2o
        typedef g2o::BlockSolver <g2o::BlockSolverTraits<6, 1>> BlockSolverType;
        typedef g2o::LinearSolverDense <BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
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

        for (size_t i = 0; i < ref_frame_->features_.size(); ++i) {
            Eigen::Vector2d pixel_ref(
                    ref_frame_->features_[i]->pixel_position_.pt.x,
                    ref_frame_->features_[i]->pixel_position_.pt.y);
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(K,
                                                                      pixel_ref,
                                                                      ref_frame_->color_ref_[i],
                                                                      ref_frame_->depth_ref_[i],
                                                                      current_frame_->color_image_,
                                                                      current_frame_->depth_image_);

            edge->setId(edge_index);
            edge->setVertex(0, vertex_pose);

            // 深度乘以归一化坐标就得到了相机坐标系下的三维点
            Eigen::Vector3d position_in_ref_cam =
                    ref_frame_->depth_ref_[i] * Eigen::Vector3d(
                            (pixel_ref[0] - K_()(0, 2)) / K_()(0, 0),
                            (pixel_ref[1] - K_()(1, 2)) / K_()(1, 1),
                            1);

            Eigen::Vector3d position_in_cur_cam =
                    vertex_pose->estimate() * position_in_ref_cam;
            Eigen::Matrix<double, 2, 1> measurements;
            measurements << ref_frame_->color_ref_[i], position_in_cur_cam[2];
            edge->setMeasurement(measurements);
            edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
            optimizer.addEdge(edge);
            edge_index++;
        }
        LOG(INFO) << "start optimization";
        optimizer.initializeOptimization();
        optimizer.optimize(G2O_OPTIMIZATION_ITERATION);
        current_frame_->SetPosition(
                vertex_pose->estimate() * ref_frame_->Pose());
        return edge_index;
    }

    bool Frontend::BuildInitialMap() {
        for (size_t i = 0; i < ref_frame_->num_good_points_; ++i) {
            if (ref_frame_->features_[i] == nullptr)
                continue;
            // create map point from depth map
            Eigen::Matrix<double, 3, 1> position_in_world =
                    ref_frame_->depth_ref_[i] * Eigen::Vector3d(
                            (ref_frame_->features_[i]->pixel_position_.pt.x -
                             K_()(0, 2)) / K_()(0, 0),
                            (ref_frame_->features_[i]->pixel_position_.pt.y -
                             K_()(1, 2)) / K_()(1, 1), 1);
            if (position_in_world[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPosition(position_in_world);
                ref_frame_->features_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
            }
        }
        map_->InsertKeyframe(ref_frame_);
        //backend_->UpdateMap();
        LOG(INFO) << "Build initial map with "
                  << current_frame_->num_good_points_ << " points.";
        return true;
    }

    bool Frontend::AddNewMapPoints() {
        Sophus::SE3 ref_Twc = ref_frame_->Pose().inverse();
        for (size_t i = 0; i < ref_frame_->num_good_points_; ++i) {
            if (ref_frame_->features_[i] == nullptr)
                continue;
            // create map point from depth map
            Eigen::Vector3d position_in_camera =
                    ref_frame_->depth_ref_[i] * Eigen::Vector3d(
                            (ref_frame_->features_[i]->pixel_position_.pt.x -
                             K_()(0, 2)) / K_()(0, 0),
                            (ref_frame_->features_[i]->pixel_position_.pt.y -
                             K_()(1, 2)) / K_()(1, 1), 1);
            if (position_in_camera[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPosition(ref_Twc * position_in_camera);
                ref_frame_->features_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
            }
        }
        LOG(INFO) << "Insert " << ref_frame_->num_good_points_ << " map points";
        map_->InsertKeyframe(ref_frame_);
        //backend_->UpdateMap();
        LOG(INFO) << "There are " << map_->GetAllLandmarks().size()
                  << " map points now";
        return true;
    }

}