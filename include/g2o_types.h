#ifndef G2O_TYPES_H
#define G2O_TYPES_H

#include "common_include.h"
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "util/get_pixel_value.h"
#include "util/tukeys_biweight.h"

namespace rgbd_slam {
    class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
    public:
        virtual void setToOriginImpl() override {
            _estimate = Sophus::SE3d();
        }

        /// left multiplication on SE3
        virtual void oplusImpl(const double *update) override {
            Eigen::Matrix<double, 6, 1> update_eigen;
            update_eigen
                    << update[0], update[1], update[2],
                    update[3], update[4], update[5];
            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        }

        virtual bool read(std::istream &in) override {}

        virtual bool write(std::ostream &out) const override {}
    };

    /// 定义仅优化位姿的一元边，2表示观测值的维度，Vec2表示观测值的数据类型是一个2×1的向量，
    /// VertexPose表示定点的数据类型
    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge
            <2, Eigen::Matrix<double, 2, 1>, VertexPose
            > {
    public:

        EdgeProjectionPoseOnly(
                const Eigen::Matrix<double, 3, 3> &K,
                const Eigen::Vector2d &pixel_ref,
                const double &color_ref,
                const double &depth_ref,
                const cv::Mat &current_image,
                const cv::Mat &current_depth) : K_(K),
                                                current_image_(
                                                        current_image),
                                                current_depth_(
                                                        current_depth) {
            point_position_ref = depth_ref *
                                 Eigen::Vector3d(
                                         (pixel_ref[0] -
                                          K_(0, 2)) /
                                         K_(0, 0),
                                         (pixel_ref[1] -
                                          K_(1, 2)) /
                                         K_(1, 1), 1);
        }

        void computeError()

        override {
            // _vertices[0]表示这条边所链接的地一个顶点，由于是一元
            // 边，因此只有_vertices[0]，若是二元边则还会
            // 存在_vertices[1]
            const VertexPose *v = dynamic_cast<VertexPose *>(_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Matrix<double, 3, 1> pixel_in_cur_cam =
                    K_ * ((T * point_position_ref) /
                          (T * point_position_ref)[2]);
            double I2 = GetPixelValue(current_image_,
                                      pixel_in_cur_cam[0],
                                      pixel_in_cur_cam[1]);
            double Z2 = current_depth_.at<unsigned short>
                    (pixel_in_cur_cam[0], pixel_in_cur_cam[1]) / DEPTH_SCALE;
            // if out of sight in current frame
            if (pixel_in_cur_cam[1]
                < 1 || pixel_in_cur_cam[1] > current_depth_
                                                     .cols - 1
                || pixel_in_cur_cam[0]
                   < 1 || pixel_in_cur_cam[0] > current_depth_
                                                        .rows - 1) {
                _error(0, 0) = 0.0;
                _error(1, 0) = 0.0;
            } else if (Z2 < MIN_DEPTH || Z2 > MAX_DEPTH) {
                _error(0, 0) = I2 - _measurement(0, 0);
                _error(1, 0) = 0;
            } else {
                _error(0, 0) = I2 - _measurement(0, 0);
                _error(1, 0) = Z2 - _measurement(1, 0);
            }
            _error(1, 0) =
                    TukeysBiweight(_error(1, 0),
                                   20);
            _error(0, 0) =
                    TukeysBiweight(_error(0, 0),
                                   50);
        }

        virtual void
        linearizeOplus()

        override // 重写线性化函数，即得到泰勒展开e(x+delta_x)=e(x)+J^T*delta_x中的J
        {
            const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d position_in_cur_cam =
                    T * point_position_ref;
            double fx = K_(0, 0);
            double fy = K_(1, 1);
            double X = position_in_cur_cam[0];
            double Y = position_in_cur_cam[1];
            double Z = position_in_cur_cam[2];
            double u_in_cur_pixel =
                    fx * position_in_cur_cam[0] /
                    position_in_cur_cam[2] + K_(0, 2);
            double v_in_cur_pixel =
                    fy * position_in_cur_cam[1] /
                    position_in_cur_cam[2] + K_(1, 2);
//std::cout<<"cur_pixel calculated"<<std::endl;
            Eigen::Matrix<double, 1, 6> J_1, J_2, J_position_xi_Z;

            double Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv =
                    Z_inv * Z_inv;
            Eigen::Matrix<double, 2, 6> J_position_xi;
            Eigen::Vector2d J_color_gradient, J_depth_gradient;

            J_position_xi(0, 0) =
                    fx * Z_inv;
            J_position_xi(0, 1) = 0;
            J_position_xi(0, 2) = -
                                          fx * X
                                  *
                                  Z2_inv;
            J_position_xi(0, 3) = -
                                          fx * X
                                  *
                                  Y * Z2_inv;
            J_position_xi(0, 4) = fx +
                                  fx * X
                                  *
                                  X * Z2_inv;
            J_position_xi(0, 5) = -
                                          fx * Y
                                  *
                                  Z_inv;

            J_position_xi(1, 0) = 0;
            J_position_xi(1, 1) =
                    fy * Z_inv;
            J_position_xi(1, 2) = -
                                          fy * Y
                                  *
                                  Z2_inv;
            J_position_xi(1, 3) = -fy -
                                  fy * Y
                                  *
                                  Y * Z2_inv;
            J_position_xi(1, 4) =
                    fy * X
                    *
                    Y * Z2_inv;
            J_position_xi(1, 5) =
                    fy * X
                    *
                    Z_inv;

            J_position_xi_Z(0, 0) = 0;
            J_position_xi_Z(0, 1) = 0;
            J_position_xi_Z(0, 2) = 1;
            J_position_xi_Z(0, 3) = -
                    Y;
            J_position_xi_Z(0, 4) =
                    X;
            J_position_xi_Z(0, 5) = 0;

            J_color_gradient = Eigen::Vector2d(
                    0.5 * (GetPixelValue(current_image_,
                                         u_in_cur_pixel + 1,
                                         v_in_cur_pixel) -
                           GetPixelValue(current_image_,
                                         u_in_cur_pixel - 1,
                                         v_in_cur_pixel)),
                    0.5 * (GetPixelValue(current_image_,
                                         u_in_cur_pixel,
                                         v_in_cur_pixel +
                                         1) -
                           GetPixelValue(current_image_,
                                         u_in_cur_pixel,
                                         v_in_cur_pixel -
                                         1)));

            J_depth_gradient = Eigen::Vector2d(
                    0.5 * (current_depth_.at<unsigned short>(int(u_in_cur_pixel) + 1,
                                                             int(v_in_cur_pixel)) -
                           current_depth_.at<unsigned short>(int(u_in_cur_pixel) - 1,
                                                             int(v_in_cur_pixel))) /
                    DEPTH_SCALE,
                    0.5 * (current_depth_.at<unsigned short>(int(u_in_cur_pixel),
                                                             int(v_in_cur_pixel) + 1) -
                           current_depth_.at<unsigned short>(int(u_in_cur_pixel),
                                                             int(v_in_cur_pixel) - 1)) /
                    DEPTH_SCALE);

            J_1 = (J_color_gradient.transpose() *
                   J_position_xi);
            J_2 = (J_depth_gradient.transpose() *
                   J_position_xi) - J_position_xi_Z;

            _jacobianOplusXi << J_1[0], J_1[1], J_1[2], J_1[3], J_1[4], J_1[5],
                    J_2[0], J_2[1], J_2[2], J_2[3], J_2[4], J_2[5];
        }

        virtual bool read(std::istream &in)

        override {
        }

        virtual bool
        write(std::ostream &out) const

        override {
        }

    private:
        Eigen::Matrix<double, 3, 1> point_position_ref;
        Eigen::Matrix<double, 3, 3> K_;
        const cv::Mat current_image_;
        const cv::Mat current_depth_;
    };
}

#endif