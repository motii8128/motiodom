#include "ekf_utils.hpp"
#include "matrix_utils.hpp"

namespace motiodom
{
    template<typename T>
    AccelAngularEKF<T>::AccelAngularEKF(float delta_time)
    {
        estimation_ = Vector3<T>(0.0, 0.0, 0.0);
        estimation_noise_ = Matrix3x3<T>(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        );

        cov_ = Matrix3x3<T>(
            0.0174*delta_time*delta_time, 0.0, 0.0,
            0.0, 0.0174*delta_time*delta_time, 0.0,
            0.0, 0.0, 0.0174*delta_time*delta_time
        );

        observation_noise_ = Matrix2x2<T>(
            0.0, 0.0,
            0.0, 0.0
        );

        kalman_gain_ = Matrix3x2<T>(
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0
        );
    }
}