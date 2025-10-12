#include "motiodom/extended_kalman_filter.hpp"

namespace motiodom
{
    ImuPostureEKF::ImuPostureEKF()
    : estimation_(Vec3(0.0, 0.0, 0.0)),
    cov_(Mat3::Identity()),
    estimation_noise_(Mat3::Identity()),
    observation_noise_(Mat2::Identity()),
    kalman_gain_(Eigen::Matrix<float, 3, 2>::Zero())
    {
        cov_(0, 0) = 0.01;
        cov_(1, 1) = 0.01;
        cov_(2, 2) = 0.01;

        estimation_noise_(0, 0) = 0.01;
        estimation_noise_(1, 1) = 0.01;
        estimation_noise_(2, 2) = 0.01;

        observation_noise_(0,0) = 0.01;
        observation_noise_(1,1) = 0.01;
    }

    Vec3 ImuPostureEKF::estimate(Vec3 &angular, Vec3 &linear_accel, const double &dt)
    {
        const auto input_matrix = getInputMatrix(angular, dt);

        const auto jacob_ = jacob(input_matrix, estimation_);
        estimation_ = predictX(input_matrix, estimation_);
        cov_ = predictCov(jacob_, cov_, estimation_noise_);
        const auto obs_model = obsModel(linear_accel);
        const auto residual = updateResidual(obs_model, estimation_);
        const auto s = updateS(cov_, observation_noise_);
        kalman_gain_ = updateKalmanGain(s, cov_);
        estimation_ = updateX(estimation_, kalman_gain_, residual);
        cov_ = updateCov(kalman_gain_, cov_);

        return estimation_;
    }

    Vec3 getInputMatrix(const Vec3& angular_velocity, const double &dt)
    {
        return Vec3(
            angular_velocity.x()* dt,
            angular_velocity.y()* dt,
            angular_velocity.z()* dt
        );
    }

    Eigen::Matrix<float, 3, 2> h()
    {
        Eigen::Matrix<float, 3, 2> h_;
        h_.setZero();
        h_(0, 0) = 1.0;
        h_(1, 1) = 1.0;
        
        return h_;
    }

    Mat3 jacob(const Vec3 &input_matrix, const Vec3 &estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        auto m_11 = 1.0 + input_matrix.y() * ((cos_roll*sin_pitch)/cos_pitch) - input_matrix.z() * ((sin_roll*sin_pitch)/cos_pitch);
        auto m_12 = input_matrix.y()*(sin_roll/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll/(cos_pitch*cos_pitch)));
        auto m_21 = -1.0*input_matrix.y()*sin_roll - input_matrix.z()*cos_roll;
        auto m_31 = input_matrix.y()*(cos_roll/cos_pitch) - input_matrix.z()*(sin_roll/cos_pitch);
        auto m_32 = input_matrix.y()*((sin_roll*sin_pitch)/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll*sin_pitch)/(cos_pitch*cos_pitch));

        Mat3 mat;
        mat.setZero();
        mat(0, 0) = m_11;
        mat(0, 1) = m_12;
        mat(1, 0) = m_21;
        mat(1, 1) = 1.0;
        mat(2, 0) = m_31;
        mat(2, 1) = m_32;

        return mat;
    }

    Vec3 predictX(const Vec3 &input_matrix, const Vec3 &estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        Vec3 est;
        est(0) = estimation.x() + input_matrix.x() + input_matrix.y()*((sin_roll*sin_pitch)/cos_pitch)+input_matrix.z()*((cos_roll*sin_pitch)/cos_pitch);
        est(1) = estimation.y() + input_matrix.y() * cos_roll - input_matrix.z()*sin_roll;
        est(2) = estimation.z() + input_matrix.z() + input_matrix.y()*(sin_roll/cos_pitch) + input_matrix.z()*(cos_roll/cos_pitch);

        return est;
    }

    Mat3 predictCov(const Mat3 &jacob, const Mat3 &cov, const Mat3 &est_noise)
    {
        Mat3 t_jacob = jacob.transpose();
        Mat3 jacob_cov = jacob * cov;

        Mat3 new_cov;
        new_cov.setZero();

        Mat3 multiplied = jacob_cov * t_jacob;

        new_cov = multiplied + est_noise;

        return new_cov;
    }

    Vec2 updateResidual(const Vec2 &obs, const Vec3 &est)
    {
        Vec2 result;
        Eigen::Matrix<float, 2, 3> h_ = h().transpose();
        Vec2 h_est = h_ * est;

        result(0) = obs.x() - h_est.x();
        result(1) = obs.y() - h_est.y();

        return result;
    }

    Mat2 updateS(const Mat3 &cov_, const Mat2 &obs_noise)
    {
        Eigen::Matrix<float, 2, 3> h_ = h().transpose();
        Eigen::Matrix<float, 2, 3> h_cov_ = h_ * cov_;
        Mat2 convert_cov_ = h_cov_ * h();

        return obs_noise + convert_cov_;
    }

    Eigen::Matrix<float, 3, 2> updateKalmanGain(const Mat2 &s, const Mat3 &cov)
    {
        auto h_ = h();

        Mat2 inverse_s = s.inverse();

        Eigen::Matrix<float, 3, 2> cov_and_h = cov * h_;

        return cov_and_h * inverse_s;
    }

    Vec3 updateX(const Vec3 &est, const Eigen::Matrix<float, 3, 2> &kalman_gain_, const Vec2 &residual)
    {
        Vec3 kalman_res = kalman_gain_ * residual;

        Vec3 result;
        result.setZero();

        result(0) = est.x() + kalman_res.x();
        result(1) = est.y() + kalman_res.y();
        result(2) = est.z() + kalman_res.z();

        return result;
    }

    Mat3 updateCov(const Eigen::Matrix<float, 3, 2> &kalman_gain, const Mat3 &cov)
    {
        Mat3 i;
        i.setIdentity();

        Eigen::Matrix<float, 2, 3> h_ = h().transpose();

        Mat3 kalman_h = kalman_gain * h_;

        Mat3 i_k_h = i - kalman_h;

        return i_k_h * cov;
    }

    Vec2 obsModel(const Vec3 &linear_accel)
    {
        Vec2 model;

        if(linear_accel.z() == 0.0)
        {
            if(linear_accel.y() > 0.0)
            {
                model(0) = acos(-1.0) / 2.0;
            }
            else
            {
                model(0) = -1.0 * acos(-1.0) / 2.0;
            }
        }
        else
        {
            model(0) = atan2(linear_accel.y(), linear_accel.z());
        }

        if(sqrt(linear_accel.y()*linear_accel.y() + linear_accel.z()*linear_accel.z()) == 0.0)
        {
            if(-1.0*linear_accel.x() > 0.0)
            {
                model(1) = acos(-1.0) / 2.0;
            }
            else
            {
                model(1) = -1.0*acos(-1.0) / 2.0;
            }
        }
        else
        {
            model(1) = atan2(-linear_accel.x(), sqrt(linear_accel.y()*linear_accel.y() + linear_accel.z()*linear_accel.z()));
        }

        return model;
    }
}