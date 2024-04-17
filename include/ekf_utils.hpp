#ifndef EKF_UTILS_HPP_
#define EKF_UTILS_HPP_

#include "matrix_utils.hpp"

#include <cmath>

namespace motiodom
{
    template<typename T>
    class AccelAngularEKF
    {
        public:
        AccelAngularEKF(float delta_time);

        Vector3<T> run(
            Vector3<T> angular_velocity,
            Vector3<T> linear_accel,
            float delta_time);

        private:
        Matrix2x3<T> h()
        {
            return Matrix2x3<T>(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0
            );
        }
        Matrix3x3<T> calc_jacob(Vector3<T> input_matrix)
        {
            auto cos_roll = cos(estimation_.x);
            auto sin_roll = sin(estimation_.x);
            auto cos_pitch = cos(estimation_.y);
            auto sin_pitch = sin(estimation_.y);

            auto m_11 = 1.0 + input_matrix.y*((cos_roll*sin_pitch)/cos_pitch) - input_matrix.z * ((sin_roll*sin_pitch)/cos_pitch);
            auto m_12 = input_matrix.y*(sin_roll/(cos_pitch*cos_pitch))+input_matrix.z*((cos_roll/(cos_pitch*cos_pitch)));
            auto m_21 = -1.0*input_matrix.y*sin_roll - input_matrix.z*cos_roll;
            auto m_31 = input_matrix.y*(cos_roll/cos_pitch) - input_matrix.z*(sin_roll/cos_pitch);
            auto m_32 = input_matrix.y*((sin_roll*sin_pitch)/(cos_pitch*cos_pitch))+input_matrix.z*((cos_roll*sin_pitch)/(cos_pitch*cos_pitch));

            return Matrix3x3<T>(
                m_11, m_12, 0.0,
                m_21, 1.0, 0.0,
                m_31, m_32, 0.0
            );
        }
        void predict_x(Vector3<T> input_matrix)
        {
            auto cos_roll = cos(estimation_.x);
            auto sin_roll = sin(estimation_.x);
            auto cos_pitch = cos(estimation_.y);
            auto sin_pitch = sin(estimation_.y);

            estimation_.x = estimation_.x + input_matrix.x + input_matrix.y*((sin_roll*sin_pitch)/cos_pitch)+input_matrix.z*((cos_roll*sin_pitch)/cos_pitch);
            estimation_.y = estimation_.y + input_matrix.y * cos_roll - input_matrix.z*sin_roll;
            estimation_.z = estimation_.z + input_matrix.z + input_matrix.y*(sin_roll/cos_pitch) + input_matrix.z*(cos_roll/cos_pitch);
        }
        void predict_cov(Matrix3x3<T> jacob)
        {
            auto t_jacob = transpose_3x3(jacob);
            auto jac_cov = multiply(jacob, cov_);
            cov_ = multiply(jac_cov, t_jacob) + estimation_noise_;
        }
        Vector2<T> update_residual(Vector2<T> observation)
        {
            Vector2<T> result;
            Matrix2x3<T> h = h();
            Vector2<T> h_est = multiply(h, estimation_);
            result.x = observation.x - h_est.x;
            result.y = observation.y - h_est.y;

            return result;
        }
        Matrix2x2<T> update_s();
        Matrix3x2<T> update_kalman_gain(Matrix2x2<T> s);
        void update_x(Vector2<T> residual);
        void update_cov();

        Vector3<T> estimation_;
        Matrix3x3<T> cov_;
        Matrix3x3<T> estimation_noise_;
        Matrix2x2<T> observation_noise_;
        Matrix3x2<T> kalman_gain_;
    };
}

#endif