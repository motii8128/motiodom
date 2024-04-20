#ifndef EKF_UTILS_HPP_
#define EKF_UTILS_HPP_

#include "matrix_utils.hpp"

#include <numbers>
#include <cmath>
#include <chrono>

namespace motiodom
{
    template<typename T>
    struct AccelAngularEKF
    {
        Vector3<T> est;
        Matrix3x3<T> cov;
        Matrix3x3<T> est_noise;
        Matrix2x2<T> obs_noise;
        Matrix3x2<T> k_gain;
    };

    template<typename T>
    AccelAngularEKF<T> init_ekf6(float delta_time)
    {
        AccelAngularEKF<T> ekf6_;

        ekf6_.est = Vector3<T>(0.0, 0.0, 0.0);
        ekf6_.cov = Matrix3x3<T>(
                    0.0174*delta_time*delta_time, 0.0, 0.0,
                    0.0, 0.0174*delta_time*delta_time, 0.0,
                    0.0, 0.0, 0.0174*delta_time*delta_time);

        ekf6_.est_noise = Matrix3x3<T>(
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);

        ekf6_.obs_noise = Matrix2x2<T>(
                    0.0, 0.0,
                    0.0, 0.0);

        ekf6_.k_gain = Matrix3x2<T>(
                    0.0, 0.0,
                    0.0, 0.0,
                    0.0, 0.0);
    }

    template<typename T>
    Matrix2x3<T> h()
    {
        return Matrix2x3<T>(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0
        );
    }
    template<typename T>
    Matrix3x3<T> calc_jacob(Vector3<T> input_matrix, Vector3<T> estimation_)
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
    template<typename T>
    Vector3<T> predict_x(Vector3<T> input_matrix, Vector3<T> estimation_)
        {
            auto cos_roll = cos(estimation_.x);
            auto sin_roll = sin(estimation_.x);
            auto cos_pitch = cos(estimation_.y);
            auto sin_pitch = sin(estimation_.y);

            Vector3<T> estimation;
            estimation.x = estimation_.x + input_matrix.x + input_matrix.y*((sin_roll*sin_pitch)/cos_pitch)+input_matrix.z*((cos_roll*sin_pitch)/cos_pitch);
            estimation.y = estimation_.y + input_matrix.y * cos_roll - input_matrix.z*sin_roll;
            estimation.z = estimation_.z + input_matrix.z + input_matrix.y*(sin_roll/cos_pitch) + input_matrix.z*(cos_roll/cos_pitch);

            return estimation;
        }
    template<typename T>
    Matrix3x3<T> predict_cov(Matrix3x3<T> jacob, Matrix3x3<T> cov_, Matrix3x3<T> estimation_noise_)
    {
        auto t_jacob = transpose_3x3<T>(jacob);
        auto jac_cov = multiply<T>(jacob, cov_);
        Matrix3x3<T> cov;
        cov = multiply<T>(jac_cov, t_jacob) + estimation_noise_;

        return cov;
    }

    template<typename T>    
    Vector2<T> update_residual(Vector2<T> observation, Vector3<T> estimation_)
    {
            Vector2<T> result;
            Matrix2x3<T> h_ = h<T>();
            Vector2<T> h_est = multiply(h_, estimation_);
            result.x = observation.x - h_est.x;
            result.y = observation.y - h_est.y;

            return result;
    }

    template<typename T>
    Matrix2x2<T> update_s(Matrix3x3<T> cov_, Matrix2x2<T> observation_noise_)
    {
            Matrix2x2<T> converted = to_2x2<T>(cov_);

            return add_2x2_2x2<T>(observation_noise_, converted);
    }
    template<typename T>
    Matrix3x2<T> update_kalman_gain(Matrix2x2<T> s, Matrix3x3<T> cov_)
    {
        Matrix2x3<T> new_h = h<T>();
        auto t_h = transpose_2x3<T>(new_h);
        auto inv_s = inverse_2x2<T>(s);

        auto cov_t_h = multiply<T>(cov_, t_h);

        return multiply<T>(cov_t_h, inv_s);
    }

    template<typename T>
    Vector3<T> update_x(Vector3<T> estimation_, Matrix3x2<T> kalman_gain_, Vector2<T> residual)
    {
        Vector3<T> kg_res = multiply<T>(kalman_gain_, residual);

        return Vector3<T>(
            estimation_.x + kg_res.x,
            estimation_.y + kg_res.y,
            estimation_.z + kg_res.z);
    }

    template<typename T>
    Matrix3x3<T> update_cov(Matrix3x2<T> kalman_gain_, Matrix3x3<T> cov_)
    {
        Matrix3x3<T> i = Matrix3x3<T>(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0);

        Matrix2x3<T> new_h = h<T>();

        Matrix3x3<T> k_h = multiply<T>(kalman_gain_, new_h);

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

        return multiply<T>(i_k_h_, cov_);
    }


    template<typename T>
    Vector2<T> obs_model_6(Vector3<T> linear_accel)
    {
        T x_ = 0.0;
        T y_ = 0.0;

        if(linear_accel.z == 0.0)
        {
            if(linear_accel.y > 0.0)
            {
                x_ = acos(-1.0) / 2.0;
            }
            else
            {
                x_ = -1.0*acos(-1.0) / 2.0;
            }
        }
        else
        {
            x_ = atan(linear_accel.y / linear_accel.z);
        }

        if(sqrt(linear_accel.y*linear_accel.y + linear_accel.z*linear_accel.z) == 0.0)
        {
            if(-1.0*linear_accel.x > 0.0)
            {
                y_ = acos(-1.0) / 2.0;
            }
            else
            {
                y_ = -1.0*acos(-1.0) / 2.0;
            }
        }
        else
        {
            y_ = (-1.0*linear_accel.x) / atan(sqrt(linear_accel.y*linear_accel.y+linear_accel.z*linear_accel.z));
        }

        return Vector2<T>(
            x_,
            y_
        );
    }

    template<typename T>
    T to_radian(T degree)
    {
        auto pi = acos(-1.0);


        return (degree*pi)/180.0;
    }
}

#endif