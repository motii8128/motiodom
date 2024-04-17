#include "matrix_utils.hpp"

namespace motiodom
{
    template<typename T>
    Matrix3x3<T>::Matrix3x3(T m11_, T m12_, T m13_, T m21_, T m22_, T m23_, T m31_, T m32_, T m33_):
    m11(m11_),m12(m12_),m13(m13_),
        m21(m21_),m22(m22_),m23(m23_),
        m31(m31_),m32(m32_),m33(m33_){}

    template<typename T>
    Matrix3x3<T> Matrix3x3<T>::transpose()
    {
        return Matrix3x3<T>(
            m11, m21, m31,
            m12, m22, m23,
            m13, m23, m33,
        );
    }

    template<typename T>
    Matrix2x2<T>::Matrix2x2(T m11_, T m12_, T m21_, T m22_):
        m11(m11_),m12(m12_), 
        m21(m21_), m22(m22_){}

    template<typename T>
    Matrix2x2<T> Matrix2x2<T>::transpose()
    {
        return Matrix2x2<T>(
            m11, m21,
            m12, m22
        );
    }
}