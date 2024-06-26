#ifndef EKF6_UTILS_HPP_
#define EKF6_UTILS_HPP_

#include "matrix_utils.hpp"

#include <numbers>
#include <cmath>
#include <chrono>

namespace motiodom
{
    struct Axis6EKF
    {
        Vector3 est;
        Matrix3x3 cov;
        Matrix3x3 est_noise;
        Matrix2x2 obs_noise;
        Matrix3x2 k_gain;

        Axis6EKF(float delta_time = 0.01);
    };


    Matrix2x3 h();
    
    Matrix3x3 calc_jacob(Vector3 input_matrix, Vector3 estimation_);

    Vector3 predict_x(Vector3 input_matrix, Vector3& estimation_);

    Matrix3x3 predict_cov(Matrix3x3 jacob, Matrix3x3 cov_, Matrix3x3 estimation_noise_);

    Vector2 update_residual(Vector2 observation, Vector3 estimation_);

    Matrix2x2 update_s(Matrix3x3 cov_, Matrix2x2 observation_noise_);
    
    Matrix3x2 update_kalman_gain(Matrix2x2 s, Matrix3x3 cov_);

    Vector3 update_x(Vector3 estimation_, Matrix3x2 kalman_gain_, Vector2 residual);

    Matrix3x3 update_cov(Matrix3x2 kalman_gain_, Matrix3x3 cov_);

    Vector2 obs_model_6(Vector3 linear_accel);

    template<typename T>
    T to_radian(T degree);
}

#endif