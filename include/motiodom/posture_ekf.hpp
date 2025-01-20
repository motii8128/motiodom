#ifndef MOTISLAM_POSTUER_EKF_HPP_
#define MOTISLAM_POSTUER_EKF_HPP_

#include "types.hpp"

namespace motiodom
{
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
        Vec3 estimate(const Vec3 &angular, const Vec3 &linear_accel, const double &dt);

        private:
        Mat3x3 cov_;
        Mat3x3 estimation_noise_;
        Mat2x2 observation_noise_;
        Mat3x2 kalman_gain_;
        Vec3 estimation_;
    };
    Vec3 getInputMatrix(const Vec3& angular_velocity, const double &dt);

    Mat3x2 h();

    Mat3x3 jacob(const Vec3 &input_matrix, const Vec3 &estimation);

    Vec3 predictX(const Vec3 &input_matrix, const Vec3 &estimation);

    Mat3x3 predictCov(const Mat3x3 &jacob, const Mat3x3 &cov, const Mat3x3 &est_noise);

    Vec2 updateResidual(const Vec2 &obs, const Vec3 &est);

    Mat2x2 updateS(const Mat3x3 &cov_, const Mat2x2 &obs_noise);

    Mat3x2 updateKalmanGain(const Mat2x2 &s, const Mat3x3 &cov);

    Vec3 updateX(const Vec3 &est, const Mat3x2 &kalman_gain_, const Vec2 &residual);

    Mat3x3 updateCov(const Mat3x2 &kalman_gain, const Mat3x3 &cov);

    Vec2 obsModel(const Vec3 &linear_accel);
}

#endif