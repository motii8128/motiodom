#include "motiodom/unscented_kalman_filter.hpp"

namespace motiodom
{
    UnscentedKalmanFilter::UnscentedKalmanFilter()
    {
        x_ = Vec8::Zero();
        P_ = Mat8::Identity() * 1e-2;

        sigma_ax_ = 0.2;
        sigma_ay_ = 0.2;
        sigma_gyro_ = 0.01;
        sigma_bg_rw_ = 1e-5;
        sigma_ba_rw_ = 1e-4;
    }

    UnscentedKalmanFilter::UnscentedKalmanFilter(const UKFConfig& cfg)
    {
        
    }
}