#ifndef MOTIODOM_EKF_HPP_
#define MOTIODOM_EKF_HPP_

#include "common.hpp"

namespace motiodom
{
    struct ImuData
    {
        float ax, ay, gyro;
    };

    struct UKFConfig
    {
        float alpha = 1e-3;
        float beta = 2.0;
        float kappa = 0.0;

        Mat8 Q = Mat8::Zero();
        Mat3 R = Mat3::Zero();
    };

    class UnscentedKalmanFilter
    {
        public:
        UnscentedKalmanFilter();
        UnscentedKalmanFilter(const UKFConfig& cfg);

        void predict(const ImuData& imu, const float& dt);

        void update(const Vec3& obs);

        Vec8 get_state();

        private:
        int n_;
        double lambda_;
        double c_;
        std::vector<double> Wm_, Wc_;

        Vec8 x_;
        Mat8 P_;
        Mat8 Q_;
        Mat3 R_;
        UKFConfig config_;

        // Preallocated sigma arrays (fixed size 2n+1 = 17)
        std::array<Vec8, 17> sigma_;
        std::array<Vec8, 17> sigma_f_; // after process
        std::array<Vec3, 17> sigma_z_;

        // indices of angles to treat specially
        std::vector<int> angleIdx_{2}; // theta index in state
        std::vector<int> angleIdxMeas_{2}; // theta index in measurement (z: [x,y,theta])

        void setSigmaWeights();
        void computeSigmaPoints();
    };

    Vec8 processModel(const Vec8 &state, const ImuData &u, double dt);
}

#endif