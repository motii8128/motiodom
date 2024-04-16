#ifndef EKF_UTILS_HPP_
#define EKF_UTILS_HPP_

#include "matrix_utils.hpp"

namespace imu_localizer_cpp
{
    template<typename T>
    class AccelAngularEKF
    {
        Vector3<T> estimation;
        Matrix3x3<T> cov;
    };
}

#endif