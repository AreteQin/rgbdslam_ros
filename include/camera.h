#pragma once
#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"

namespace rgbd_slam
{

    // rgbd camera
    class Camera
    {
    public:
        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0; // camera intrinsics

        Camera();

        Camera(double fx, double fy, double cx, double cy)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

        // return intrinsic matrix
        Eigen::Matrix<double, 3, 3> K() const
        {
            Eigen::Matrix<double, 3, 3> intrinsic_matrix;
            intrinsic_matrix << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
            return intrinsic_matrix;
        }

        // 3D point coordinate transform: world, camera, pixel
        Eigen::Matrix<double, 3, 1> World2Camera(const Eigen::Matrix<double, 3, 1> &position_in_world, const Sophus::SE3d &T_c_w);

        Eigen::Matrix<double, 3, 1> Camera2World(const Eigen::Matrix<double, 3, 1> &position_in_camera, const Sophus::SE3d &T_c_w);

        Eigen::Matrix<double, 2, 1> Camera2Pixel(const Eigen::Matrix<double, 3, 1> &position_in_camera);

        Eigen::Matrix<double, 3, 1> Pixel2Camera(const Eigen::Matrix<double, 2, 1> &position_in_pixel, double depth);

        Eigen::Matrix<double, 3, 1> Pixel2World(const Eigen::Matrix<double, 2, 1> &position_in_pixel, const Sophus::SE3d &T_c_w, double depth);

        Eigen::Matrix<double, 2, 1> World2Pixel(const Eigen::Matrix<double, 3, 1> &position_in_world, const Sophus::SE3d &T_c_w);
    };

}
#endif