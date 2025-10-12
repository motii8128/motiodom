#ifndef MOTIODOM_EKF_HPP_
#define MOTIODOM_EKF_HPP_

#include "common.hpp"

namespace motiodom
{
    struct ImuData
    {
        float dt;
        float ax, ay, gyro;
    };

    class EKF
    {
        public:
        EKF();

        void predict(const ImuData& imu);

        void update(const float& x, const float& y, const float& yaw);

        Vec8 get_x();

        private:
        Vec8 x_;
        Mat8 P_;

        float sigma_ax_, sigma_ay_, sigma_gyro_;
        float sigma_bg_rw_, sigma_ba_rw_;
    };
}

#endif