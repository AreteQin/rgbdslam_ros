//
// Created by qin on 3/7/22.
//

#ifndef SRC_CERES_TYPES_H
#define SRC_CERES_TYPES_H

#include <ceres/ceres.h>

#include <utility>
#include "common_include.h"
#include "util/get_pixel_value.h"
#include "util/tukeys_biweight.h"

namespace rgbd_slam {
    class DepthIlluminationError : public ceres::SizedCostFunction<1, 6> {
    public:
        DepthIlluminationError(Eigen::Matrix<double, 3, 3> K,
                               const Eigen::Vector2d &pixel_ref,
                               const double &color_ref,
                               const double &depth_ref,
                               cv::Mat current_image,
                               cv::Mat current_depth) : K_(std::move(K)),
                                                        color_ref_(color_ref),
                                                        current_image_(std::move(current_image)),
                                                        current_depth_(std::move(current_depth)) {
            point_position_ref_ = depth_ref * Eigen::Vector3d((pixel_ref[0] - K_(0, 2)) / K_(0, 0),
                                                              (pixel_ref[1] - K_(1, 2)) / K_(1, 1),
                                                              1);
        }

        ~DepthIlluminationError() override = default;

        bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {

            Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3(*parameters);
            Sophus::SE3d T = Sophus::SE3d::exp(se3);
            Eigen::Vector3d position_in_cur_cam = T * point_position_ref_;
            double u_in_cur_pixel = K_(0, 0) * position_in_cur_cam[0] / position_in_cur_cam[2] + K_(0, 2);
            double v_in_cur_pixel = K_(1, 1) * position_in_cur_cam[1] / position_in_cur_cam[2] + K_(1, 2);

            double I2 = GetPixelValue(current_image_, u_in_cur_pixel, v_in_cur_pixel);
            double Z2 = current_depth_.at<unsigned short>(u_in_cur_pixel, v_in_cur_pixel);
            // if out of sight in current frame
            if (v_in_cur_pixel < 1 || v_in_cur_pixel > current_depth_.cols - 1
                || u_in_cur_pixel < 1 || u_in_cur_pixel > current_depth_.rows - 1) {
                residuals[0] = 0.0;
                residuals[1] = 0.0;
            } else if (Z2 == 0) {
                residuals[0] = I2 - color_ref_;
                residuals[1] = 0;
            } else {
                residuals[0] = I2 - color_ref_;
                residuals[1] = 10 * (Z2 - position_in_cur_cam[2]) / DEPTH_SCALE;
            }

            residuals[0] = TukeysBiweight(residuals[0], 10);
            residuals[1] = TukeysBiweight(residuals[1], 10);

//            if (jacobians != nullptr) {
//                if (jacobians[0] != nullptr) {
//                    double X = position_in_cur_cam[0];
//                    double Y = position_in_cur_cam[1];
//                    double Z = position_in_cur_cam[2];
//                    Eigen::Matrix<double, 1, 6> J_1, J_2, J_position_xi_Z;
//
//                    double Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;
//                    Eigen::Matrix<double, 2, 6> J_position_xi;
//                    Eigen::Vector2d J_color_gradient, J_depth_gradient;
//
//                    Eigen::Map<Eigen::Matrix<double, 6, 2>> J(jacobians[0]);
//
//                    J_position_xi(0, 0) = K_(0,0) * Z_inv;
//                    J_position_xi(0, 1) = 0;
//                    J_position_xi(0, 2) = -K_(0,0) * X * Z2_inv;
//                    J_position_xi(0, 3) = -K_(0,0) * X * Y * Z2_inv;
//                    J_position_xi(0, 4) = K_(0,0) + K_(0,0) * X * X * Z2_inv;
//                    J_position_xi(0, 5) = -K_(0,0) * Y * Z_inv;
//                    J_position_xi(1, 0) = 0;
//                    J_position_xi(1, 1) = K_(1,1) * Z_inv;
//                    J_position_xi(1, 2) = -K_(1,1) * Y * Z2_inv;
//                    J_position_xi(1, 3) = -K_(1,1) - K_(1,1) * Y * Y * Z2_inv;
//                    J_position_xi(1, 4) = K_(1,1) * X * Y * Z2_inv;
//                    J_position_xi(1, 5) = K_(1,1) * X * Z_inv;
//                    J_position_xi_Z(0, 0) = 0;
//                    J_position_xi_Z(0, 1) = 0;
//                    J_position_xi_Z(0, 2) = 1;
//                    J_position_xi_Z(0, 3) = -Y;
//                    J_position_xi_Z(0, 4) = X;
//                    J_position_xi_Z(0, 5) = 0;
//
//                    J_color_gradient = Eigen::Vector2d(
//                            0.5 * (GetPixelValue(current_image_, u_in_cur_pixel + 1, v_in_cur_pixel) -
//                                   GetPixelValue(current_image_, u_in_cur_pixel - 1, v_in_cur_pixel)),
//                            0.5 * (GetPixelValue(current_image_, u_in_cur_pixel, v_in_cur_pixel + 1) -
//                                   GetPixelValue(current_image_, u_in_cur_pixel, v_in_cur_pixel - 1)));
//
//                    J_depth_gradient = Eigen::Vector2d(
//                            0.5 * (current_depth_.at<unsigned short>(int(u_in_cur_pixel) + 1, int(v_in_cur_pixel)) -
//                                   current_depth_.at<unsigned short>(int(u_in_cur_pixel) - 1, int(v_in_cur_pixel))),
//                            0.5 * (current_depth_.at<unsigned short>(int(u_in_cur_pixel), int(v_in_cur_pixel) + 1) -
//                                   current_depth_.at<unsigned short>(int(u_in_cur_pixel), int(v_in_cur_pixel) - 1)));
//
//                    J_1 = (J_color_gradient.transpose() * J_position_xi);
//                    J_2 = (J_depth_gradient.transpose() * J_position_xi) - J_position_xi_Z;
//
//                    J << J_1[0], J_1[1], J_1[2], J_1[3], J_1[4], J_1[5],
//                            J_2[0], J_2[1], J_2[2], J_2[3], J_2[4], J_2[5];
//                }
//            }
            return true;
        }

    private:
        double color_ref_;
        Eigen::Matrix<double, 3, 1> point_position_ref_;
        Eigen::Matrix<double, 3, 3> K_;
        const cv::Mat current_image_;
        const cv::Mat current_depth_;
    };
}
#endif //SRC_CERES_TYPES_H
