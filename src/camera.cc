#include "camera.h"
#include "common_include.h"

namespace rgbd_slam
{

    Camera::Camera(){} // intention?


    Eigen::Matrix<double, 3, 1> Camera::World2Camera(const Eigen::Matrix<double, 3, 1> &position_in_world, const Sophus::SE3d &T_c_w)
    {
        return T_c_w * position_in_world;
    }

    Eigen::Matrix<double, 3, 1> Camera::Camera2World(const Eigen::Matrix<double, 3, 1> &position_in_camera, const Sophus::SE3d &T_c_w)
    {
        return T_c_w.inverse() * position_in_camera;
    }

    Eigen::Matrix<double, 2, 1> Camera::Camera2Pixel(const Eigen::Matrix<double, 3, 1> &position_in_camera)
    {
        return Eigen::Matrix<double, 2, 1>(
            fx_ * position_in_camera(0, 0) / position_in_camera(2, 0) + cx_,
            fy_ * position_in_camera(1, 0) / position_in_camera(2, 0) + cy_);
    }

    Eigen::Matrix<double, 3, 1> Camera::Pixel2Camera(const Eigen::Matrix<double, 2, 1> &position_in_pixel, double depth = 1)
    {
        return Eigen::Matrix<double, 3, 1>(
            (position_in_pixel(0, 0) - cx_) * depth / fx_,
            (position_in_pixel(1, 0) - cy_) * depth / fy_,
            depth);
    }

    Eigen::Matrix<double, 3, 1> Camera::Pixel2World(const Eigen::Matrix<double, 2, 1> &position_in_pixel, const Sophus::SE3d &T_c_w, double depth)
    {
        return Camera2World(Pixel2Camera(position_in_pixel, depth), T_c_w);
    }

    Eigen::Matrix<double, 2, 1> Camera::World2Pixel(const Eigen::Matrix<double, 3, 1> &position_in_world, const Sophus::SE3d &T_c_w)
    {
        return Camera2Pixel(World2Camera(position_in_world, T_c_w));
    }

}