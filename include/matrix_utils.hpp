#ifndef MATRIX_UTILS_HPP_
#define MATRIX_UTILS_HPP_

namespace imu_localizer_cpp
{
    template<typename T>
    struct Vector3
    {
        T x, y, z;

        Vector3(T x_, T y_, T z_):x(x_),y(y_),z(z_){}
    };

    template<typename T>
    class Matrix3x3
    {
        public:
        Matrix3x3(T m11_, T m12_, T m13_, T m21_, T m22_, T m23_, T m31_, T m32_, T m33_):
        m11(m11_),m12(m12_),m13(m13_),
        m21(m21_),m22(m22_),m23(m23_),
        m31(m31_),m32(m32_),m33(m33_){}

        Matrix3x3<T> transpose()
        {
            Matrix3x3 transposed = Matrix3x3<T>(m11, m21, m31,
                                        m12, m22, m32,
                                        m13, m23, m33);

            return transposed;
        }

        private:
        T m11, m12, m13, m21, m22, m23, m31, m32, m33;
    };
}

#endif 