#ifndef EKF_UTILS_HPP_
#define EKF_UTILS_HPP_

#include "matrix_utils.hpp"

namespace motiodom
{
    template<typename T>
    class AccelAngularEKF
    {
        public:
        AccelAngularEKF(float delta_time);


        private:
        Matrix2x3<T> h();
        Matrix3x3<T> calc_jacob(Vector3<T> input_matrix);
        void predict_x(Vector3<T> input_matrix);
        void predict_cov(Matrix3x3<T> jacob);
        Vector2<T> update_residual(Vector2<T> observation);
        Matrix2x2<T> update_s();
        Matrix3x2<T> update_kalman_gain(Matrix2x2<T> s);
        void update_x(Vector2<T> residual);
        void update_cov();

        Vector3<T> estimation_;
        Matrix3x3<T> cov_;
        Matrix3x3<T> estimation_noise_;
        Matrix2x2<T> observation_noise_;
        Matrix3x2<T> kalman_gain_;
    };
}

#endif