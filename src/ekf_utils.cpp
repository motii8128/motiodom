#include "ekf_utils.hpp"
#include "matrix_utils.hpp"

namespace motiodom
{
    template<typename T>
    void AccelAngularEKF<T>::init(float delta_time)
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

    template<typename T>
    Vector3<T> AccelAngularEKF<T>::run(
        Vector3<T> angular_velocity,
        Vector3<T> linear_accel,
        float delta_time
    )
    {
        Vector3<T> input_matrix = Vector3<T>(
            angular_velocity.x*delta_time,
            angular_velocity.y*delta_time,
            angular_velocity.z*delta_time
        );

        estimation_noise_ = Matrix3x3<T>(
            0.0174*delta_time*delta_time, 0.0, 0.0,
            0.0, 0.0174*delta_time*delta_time, 0.0,
            0.0, 0.0, 0.0174*delta_time*delta_time);

        observation_noise_ = Matrix2x2<T>(
            delta_time*delta_time, 0.0,
            0.0, delta_time);

        Matrix3x3<T> jacob = calc_jacob(input_matrix);

        predict_x(input_matrix);

        predict_cov(jacob);

        Vector2<T> obs = obs_model_6(linear_accel);

        Vector2<T> residual = update_residual(obs);

        Matrix2x2<T> s = update_s();

        update_kalman_gain(s);

        update_x(residual);
        update_cov();

        return estimation_;
    }
}