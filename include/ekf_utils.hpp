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
        Matrix2x2<T> update_s()
        {
            Matrix2x2<T> converted = to_2x2(cov_);

            return add_2x2_2x2(observation_noise_, converted);
        }
        Matrix3x2<T> update_kalman_gain(Matrix2x2<T> s)
        {
            Matrix2x3<T> new_h = h();
            auto t_h = transpose_2x3(new_h);
            auto inv_s = inverse_2x2(s);

            auto cov_t_h = multiply(cov_, t_h);

            kalman_gain_ = multiply(cov_t_h, inv_s);
        }
        void update_x(Vector2<T> residual)
        {
            Vector3<T> kg_res = multiply(kalman_gain_, residual);

            estimation_.x = estimation_.x + kg_res.x;
            estimation_.y = estimation_.y + kg_res.y;
            estimation_.z = estimation_.z + kg_res.z;
        }
        void update_cov()
        {
            Matrix3x3<T> i = Matrix3x3<T>(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0);

            Matrix2x3<T> new_h = h();

            Matrix3x3<T> k_h = multiply(kalman_gain_, new_h);

            Matrix3x3<T> i_k_h_;

            i_k_h_.m11 = i.m11 - k_h.m11;
            i_k_h_.m12 = i.m12 - k_h.m12;
            i_k_h_.m13 = i.m13 - k_h.m13;

            i_k_h_.m21 = i.m21 - k_h.m21;
            i_k_h_.m22 = i.m22 - k_h.m22;
            i_k_h_.m23 = i.m23 - k_h.m23;

            i_k_h_.m31 = i.m31 - k_h.m31;
            i_k_h_.m32 = i.m32 - k_h.m32;
            i_k_h_.m33 = i.m33 - k_h.m33;

            cov_ = multiply(i_k_h_, cov_);
        }

        Vector3<T> estimation_;
        Matrix3x3<T> cov_;
        Matrix3x3<T> estimation_noise_;
        Matrix2x2<T> observation_noise_;
        Matrix3x2<T> kalman_gain_;
    };
}

#endif