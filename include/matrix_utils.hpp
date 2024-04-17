#ifndef MATRIX_UTILS_HPP_
#define MATRIX_UTILS_HPP_

namespace motiodom
{
    template<typename T>
    struct Vector3
    {
        T x, y, z;

        Vector3(T x_, T y_, T z_);
    };

    template<typename T>
    struct Vector2
    {
        T x, y;

        Vector2(T x_, T y_);
    };
    

    template<typename T>
    struct Matrix3x3
    {
        Matrix3x3(T m11_, T m12_, T m13_, T m21_, T m22_, T m23_, T m31_, T m32_, T m33_);

        T m11, m12, m13, m21, m22, m23, m31, m32, m33;
    };

    template<typename T>
    struct Matrix2x2
    {
        Matrix2x2(T m11_, T m12_, T m21_, T m22_);

        T m11, m12, m21, m22,
    };

    template<typename T>
    struct Matrix3x2
    {
        Matrix3x2(T m11_, T m12_, T m21_, T m22_, T m31_, T m32_);
        
        T m11, m12, m21, m22, m31, m32;
    };

    template<typename T>
    struct Matrix2x3
    {
        Matrix2x3(T m11_, T m12_, T m13_, T m21_, T m22_, T m23_);

        T m11, m12, m13, m21, m22, m23;
    };

    template<typename T>
    Matrix3x3<T> transpose_3x3(Matrix3x3<T> matrix);

    template<typename T>
    Matrix2x2<T> transpose_2x2(Matrix2x2<T> matrix);

    template<typename T>
    Matrix2x3<T> transpose_3x2(Matrix3x2<T> matrix);

    template<typename T>
    Matrix3x2<T> transpose_2x3(Matrix2x3<T> matrix);
}

#endif 