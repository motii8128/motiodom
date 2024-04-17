#include "matrix_utils.hpp"

namespace motiodom
{
    template<typename T>
    Vector3<T>::Vector3(T x_, T y_, T z_):x(x_),y(y_),z(z_){}

    template<typename T>
    Vector2<T>::Vector2(T x_, T y_):x(x_),y(y_){}

    template<typename T>
    Matrix3x3<T>::Matrix3x3(T m11_, T m12_, T m13_, T m21_, T m22_, T m23_, T m31_, T m32_, T m33_):
    m11(m11_),m12(m12_),m13(m13_),
        m21(m21_),m22(m22_),m23(m23_),
        m31(m31_),m32(m32_),m33(m33_){}

    template<typename T>
    Matrix2x2<T>::Matrix2x2(T m11_, T m12_, T m21_, T m22_):
        m11(m11_),m12(m12_), 
        m21(m21_), m22(m22_){}

    template<typename T>
    Matrix3x2<T>::Matrix3x2(T m11_, T m12_, T m21_, T m22_, T m31_, T m32_):
    m11(m11_),m12(m12_),
    m21(m21_),m22(m22_),
    m31(m31_),m32(m32_){}

    template<typename T>
    Matrix2x3<T>::Matrix2x3(T m11_, T m12_, T m13_, T m21_, T m22_, T m23_):
    m11(m11_),m12(m12_),m13(m13_),
    m21(m21_),m22(m22_),m23(m23_){}

    template<typename T>
    Matrix3x3<T> transpose_3x3(Matrix3x3<T> matrix)
    {
        return Matrix3x3<T>(
            matrix.m11, matrix.m21, matrix.m31,
            matrix.m12, matrix.m22, matrix.m23,
            matrix.m13, matrix.m23, matrix.m33,
        );
    }

    template<typename T>
    Matrix2x2<T> transpose_2x2(Matrix2x2<T> matrix)
    {
        return Matrix2x2<T>(
            matrix.m11, matrix.m21,
            matrix.m12, matrix.m22
        );
    }

    
}