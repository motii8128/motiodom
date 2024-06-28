#ifndef EKF9_UTILS_HPP_
#define EKF9_UTILS_HPP_

#include "matrix_utils.hpp"
#include <cmath>

namespace motiodom
{
    class Axis9EKF
    {
        public:
        Axis9EKF();
        Vector3 run(Vector3 angular_velocity, Vector3 linear_accel, Vector3 magnet);
        
        private:
        Vector3 est;
        Matrix3x3 cov;
        Matrix3x3 est_noise;
        Matrix3x3 obs_noise;
        Matrix3x3 k_gain;
    };

    Vector3 predict_x(Vector3 prev, Vector3 angular_velocity);

    Vector3 observation_model(Vector3 linear_accel, Vector3 mag_);

    Matrix3x3 estimation_jacob(Vector3 est, Vector3 angular_velocity);

    Matrix3x3 estimation_cov(Matrix3x3 cov, Matrix3x3 est_jacob, Matrix3x3 est_noise);

    Matrix3x3 observation_cov(Matrix3x3 obs_noise, Matrix3x3 est_cov, Matrix3x3 obs_jacob);

    Matrix3x3 kalman_gain(Matrix3x3 est_cov, Matrix3x3 obs_jacob, Matrix3x3 obs_cov);

    Vector3 update_x(Vector3 est, Vector3 obs, Matrix3x3 kalman_gain);

    Matrix3x3 update_cov(Matrix3x3 kalman_gain, Matrix3x3 est_cov);
}

#endif