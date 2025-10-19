#include "motiodom/unscented_kalman_filter.hpp"

namespace motiodom
{
    UnscentedKalmanFilter::UnscentedKalmanFilter()
    {
        n_ = 8;

        Q_ = Mat8::Zero();
        R_ = Mat3::Zero();

        P_ = Mat8::Identity() * 1e-3;

        x_ = Vec8::Zero();

        setSigmaWeights();
    }

    UnscentedKalmanFilter::UnscentedKalmanFilter(const UKFConfig& cfg)
    {
        n_ = 8;

        Q_ = cfg.Q;
        R_ = cfg.R;

        P_ = Mat8::Identity() * 1e-3;

        x_ = Vec8::Zero();

        config_ = cfg;

        setSigmaWeights();
    }

    void UnscentedKalmanFilter::predict(const ImuData& imu, const float& dt)
    {
        computeSigmaPoints();

        int point_num = 2 * n_ + 1;
        for(int i = 0; i < point_num; i++)
        {
            sigma_f_[i] = processModel(sigma_[i], imu, dt);
        }
    }


    void UnscentedKalmanFilter::setSigmaWeights() 
    {
        lambda_ = config_.alpha * config_.alpha * (n_ + config_.kappa) - n_;
        c_ = n_ + lambda_;
        Wm_.assign(2*n_+1, 1.0/(2.0*c_));
        Wc_.assign(2*n_+1, 1.0/(2.0*c_));
        Wm_[0] = lambda_ / c_;
        Wc_[0] = Wm_[0] + (1 - config_.alpha * config_.alpha + config_.beta);
    }

    void UnscentedKalmanFilter::computeSigmaPoints() 
    {
        Mat8 Ps = c_ * P_;

        // Cholesky (LLT) with jitter if necessary
        Mat8 L;

        bool ok = true;
        Eigen::LLT<Mat8> llt(Ps);
        if (llt.info() == Eigen::Success) 
        {
            L = llt.matrixL();
        }
        else 
        {
            // add jitter
            Mat8 jitter = Mat8::Identity() * 1e-9;
            L = (Ps + jitter).llt().matrixL();
        }

        sigma_[0] = x_;
        for (int i=0;i<n_;++i) 
        {
            Vec8 col = L.col(i);
            sigma_[1 + i] = x_ + col;
            sigma_[1 + n_ + i] = x_ - col;
        }
    }

    Vec8 processModel(const Vec8 &state, const ImuData &u, double dt) 
    {
        Vec8 new_state = state;

        // get state value
        const auto state_x = state(0);
        const auto state_y = state(1); 
        const auto state_theta = state(2);
        const auto state_vel_x = state(3);
        const auto state_vel_y = state(4);
        const auto accel_x_bias = state(5);
        const auto accel_y_bias = state(6);
        const auto gyro_bias = state(7);


        // bias corrected accel in body frame
        const auto accel_x_corr = u.ax - accel_x_bias;
        const auto accel_y_corr = u.ay - accel_y_bias;

        const auto cos = std::cos(state_theta); 
        const auto sin = std::sin(state_theta);

        // 2D Rotation
        const auto accel_world_x = cos * accel_x_corr - sin * accel_y_corr; 
        const auto accel_world_y = sin * accel_x_corr + cos * accel_y_corr;

        // simple integration
        new_state(0) = state_x + state_vel_x * dt + 0.5 * accel_world_x * dt * dt; 
        new_state(1) = state_y + state_vel_y * dt + 0.5 * accel_world_y * dt * dt;
        new_state(2) = state_theta + (u.gyro - gyro_bias) * dt;
        new_state(3) = state_vel_x + accel_world_x * dt; 
        new_state(4) = state_vel_y + accel_world_y * dt;
        
        return new_state;
    }
}