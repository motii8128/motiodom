#ifndef MATRIX_UTILS_HPP_
#define MATRIX_UTILS_HPP_

namespace motiodom
{
    template<typename T>
    struct Vector3
    {
        T x, y, z;

        Vector3<T>(T x_, T y_, T z_);
    };

    template<typename T>
    struct Vector2
    {
        T x, y;

        Vector2<T>(T x_, T y_);
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

        T m11, m12, m21, m22;
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

    template<typename T>
    Matrix2x2<T> inverse_2x2(Matrix2x2<T> matrix);

    template<typename T>
    Matrix3x3<T> multiply(Matrix3x3<T> a, Matrix3x3<T> b)
    {
        return Matrix3x3<T>(
            a.m11*b.m11 + a.m12*b.m21 + a.m13*b.m31, a.m11*b.m12 + a.m12*b.m22 + a.m13*b.m32, a.m11*b.m13 + a.m12*b.m23 + a.m13*b.m33,
            a.m21*b.m11 + a.m22*b.m21 + a.m23*b.m31, a.m21*b.m12 + a.m22*b.m22 + a.m23*b.m32, a.m21*b.m13 + a.m22*b.m23 + a.m23*b.m33,
            a.m31*b.m11 + a.m32*b.m21 + a.m33*b.m31, a.m31*b.m12 + a.m32*b.m22 + a.m33*b.m32, a.m31*b.m13 + a.m32*b.m23 + a.m33*b.m33);
    }

    template<typename T>
    Vector2<T> multiply(Matrix2x3<T> a, Vector3<T> b)
    {
        return Vector2<T>(
            a.m11*b.x + a.m12*b.y + a.m13*b.z,
            a.m21*b.x + a.m22*b.y + a.m23*b.z);
    }

    template<typename T>
    Matrix3x2<T> multiply(Matrix3x3<T> a, Matrix3x2<T> b)
    {
        return Matrix3x2<T>(
            a.m11*b.m11+a.m12*b.m21+a.m13*b.m31, a.m11*b.m12+a.m12*b.m22+a.m13*b.m32,
            a.m21*b.m11+a.m22*b.m21+a.m23*b.m31, a.m21*b.m12+a.m22*b.m22+a.m23*b.m32,
            a.m31*b.m11+a.m32*b.m21+a.m33*b.m31, a.m31*b.m12+a.m32*b.m22+a.m33*b.m32);
    }

    template<typename T>
    Vector3<T> multiply(Matrix3x2<T> a, Vector2<T> b)
    {
        return Vector3<T>(
            a.m11*b.x + a.m12*b.y,
            a.m21*b.x + a.m22*b.y,
            a.m31*b.x + a.m32*b.y);
    } 

    template<typename T>
    Matrix3x2<T> multiply(Matrix3x2<T> a, Matrix2x2<T> b)
    {
        return Matrix3x2<T>(
            a.m11*b.m11 + a.m12*b.m21, a.m11*b.m12 + a.m12*b.m22,
            a.m21*b.m11 + a.m22*b.m21, a.m21*b.m12 + a.m22*b.m22,
            a.m31*b.m11 + a.m32*b.m21, a.m31*b.m12 + a.m32*b.m22);
    }

    template<typename T>
    Matrix3x3<T> multiply(Matrix3x2<T> a, Matrix2x3<T> b)
    {
        return Matrix3x3<T>(
            a.m11*b.m11+a.m12*b.m21, a.m11*b.m12+a.m12*b.m22, a.m11*b.m13+a.m12*b.m23,
            a.m21*b.m11+a.m22*b.m21, a.m21*b.m12+a.m22*b.m22, a.m21*b.m13+a.m22*b.m23,
            a.m31*b.m11+a.m32*b.m21, a.m31*b.m12+a.m32*b.m22, a.m31*b.m13+a.m32*b.m23);
    }

    template<typename T>
    Matrix2x2<T> to_2x2(Matrix3x3<T> matrix)
    {
        Matrix2x2<T> result;

        result.m11 = matrix.m11;
        result.m12 = matrix.m12;
        result.m21 = matrix.m21;
        result.m22 = matrix.m22;

        return result;
    }

    template<typename T>
    Matrix2x2<T> add_2x2_2x2(Matrix2x2<T> a, Matrix2x2<T> b)
    {
        Matrix2x2<T> result;

        result.m11 = a.m11 + b.m11;
        result.m12 = a.m12 + b.m12;
        result.m21 = a.m21 + b.m21;
        result.m22 = a.m22 + b.m22;

        result;
    }
}

#endif 