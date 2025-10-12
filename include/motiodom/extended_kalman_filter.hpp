#ifndef POSTUER_EKF_HPP_
#define POSTUER_EKF_HPP_

#include "common.hpp"

namespace motiodom
{
    using Mat3 = Eigen::Matrix3f;
    using Vec3 = Eigen::Vector3f;

    class ImuPostureEKF
    {
        public:
        /// @brief コンストラクタ
        ImuPostureEKF();

        /// @brief 加速度と角速度から姿勢を推定する
        /// @param angular 角速度[rad]
        /// @param linear_accel 加速度[m/s^2]
        /// @param dt 時間間隔
        /// @return オイラー角(roll, pitch, yaw)
        Vec3 estimate(Vec3 &angular, Vec3 &linear_accel, const double &dt);

        private:
        Mat3 cov_;
        Mat3 estimation_noise_;
        Mat2 observation_noise_;
        Eigen::Matrix<float,3,2> kalman_gain_;
        Vec3 estimation_;
    };
    Vec3 getInputMatrix(const Vec3& angular_velocity, const double &dt);

    Eigen::Matrix<float,3,2> h();

    Mat3 jacob(const Vec3 &input_matrix, const Vec3 &estimation);

    Vec3 predictX(const Vec3 &input_matrix, const Vec3 &estimation);

    Mat3 predictCov(const Mat3 &jacob, const Mat3 &cov, const Mat3 &est_noise);

    Point2f updateResidual(const Point2f &obs, const Vec3 &est);

    Mat2 updateS(const Mat3 &cov_, const Mat2 &obs_noise);

    Eigen::Matrix<float,3,2> updateKalmanGain(const Mat2 &s, const Mat3 &cov);

    Vec3 updateX(const Vec3 &est, const Eigen::Matrix<float,3,2> &kalman_gain_, const Point2f &residual);

    Mat3 updateCov(const Eigen::Matrix<float,3,2> &kalman_gain, const Mat3 &cov);

    Point2f obsModel(const Vec3 &linear_accel);
}

#endif