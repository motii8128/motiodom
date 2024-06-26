#include "ekf6_utils.hpp"

namespace motiodom
{
    AccelAngularEKF::AccelAngularEKF(float delta_time = 0.01):
        est(Vector3(0.0, 0.0, 0.0)), 
        cov(Matrix3x3(
            0.0174*delta_time*delta_time, 0.0, 0.0,
            0.0, 0.0174*delta_time*delta_time, 0.0,
            0.0, 0.0, 0.0174*delta_time*delta_time)),
        est_noise(Matrix3x3(
            0.0174*0.01*0.01, 0.0, 0.0, 
            0.0, 0.0174*0.01*0.01, 0.0, 
            0.0, 0.0, 0.0174*0.01*0.01)),
        obs_noise(Matrix2x2(0.0, 0.0, 0.0, 0.0)),
        k_gain(Matrix3x2(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)){}

    Matrix2x3 h()
    {
        return Matrix2x3(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0
        );
    }

    Matrix3x3 calc_jacob(Vector3 input_matrix, Vector3 estimation_)
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

            return Matrix3x3(
                m_11, m_12, 0.0,
                m_21, 1.0, 0.0,
                m_31, m_32, 0.0
            );
    }
}