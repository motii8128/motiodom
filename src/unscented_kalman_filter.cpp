#include "motiodom/ekf.hpp"

namespace motiodom
{
    EKF::EKF()
    {
        x_ = Vec8::Zero();
        P_ = Mat8::Identity() * 1e-2;

        sigma_ax_ = 0.2;
        sigma_ay_ = 0.2;
        sigma_gyro_ = 0.01;
        sigma_bg_rw_ = 1e-5;
        sigma_ba_rw_ = 1e-4;
    }

    void EKF::predict(const ImuData& imu)
    {
        if(imu.dt <= 0.0) return;

        // ｘ位置
        const auto px = x_(0);
        // y位置
        const auto py = x_(1);
        // 角度
        const auto theta = x_(2);
        // x速度
        const auto vx = x_(3);
        // y速度
        const auto vy = x_(4);
        // ジャイロバイアス
        const auto bg = x_(5);
        // x加速度バイアス
        const auto bax = x_(6);
        // y加速度バイアス
        const auto bay = x_(7);

        const auto corrected_gyro = imu.gyro - bg;
        const auto corrected_ax = imu.ax - bax;
        const auto correcred_ay = imu.ay - bay;

        
    }
}