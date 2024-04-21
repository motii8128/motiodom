#ifndef MATRIX_UTILS_HPP_
#define MATRIX_UTILS_HPP_

namespace motiodom
{
    struct Vector3
    {
        float x, y, z;

        Vector3(float x_, float y_, float z_):x(x_),y(y_),z(z_){}
    };

    struct Vector2
    {
        float x, y;

        Vector2(float x_, float y_):x(x_),y(y_){}
    };
    
    struct Matrix3x3
    {
        Matrix3x3(float m11_, float m12_, float m13_, float m21_, float m22_, float m23_, float m31_, float m32_, float m33_):
        m11(m11_),m12(m12_),m13(m13_),
        m21(m21_),m22(m22_),m23(m23_),
        m31(m31_),m32(m32_),m33(m33_){}

        float m11, m12, m13, m21, m22, m23, m31, m32, m33;
    };

    struct Matrix2x2
    {
        Matrix2x2(float m11_, float m12_, float m21_, float m22_):
        m11(m11_),m12(m12_),
        m21(m21_),m22(m22_)
        {}

        float m11, m12, m21, m22;
    };

    struct Matrix3x2
    {
        Matrix3x2(float m11_, float m12_, float m21_, float m22_, float m31_, float m32_):
        m11(m11_),m12(m12_),
        m21(m21_),m22(m22_)
        {}
        
        float m11, m12, m21, m22, m31, m32;
    };

    struct Matrix2x3
    {
        Matrix2x3(float m11_, float m12_, float m13_, float m21_, float m22_, float m23_):
        m11(m11_),m12(m12_),m13(m13_),
        m21(m21_),m22(m22_),m23(m23_)
        {}

        float m11, m12, m13, m21, m22, m23;
    };

    Matrix3x3 transpose_3x3(Matrix3x3 matrix)
    {
        return Matrix3x3(
            matrix.m11, matrix.m21, matrix.m31,
            matrix.m12, matrix.m22, matrix.m23,
            matrix.m13, matrix.m23, matrix.m33
        );
    }
    
    Matrix2x2 transpose_2x2(Matrix2x2 matrix)
    {
        return Matrix2x2(
            matrix.m11, matrix.m21,
            matrix.m12, matrix.m22
        );
    }

    Matrix2x3 transpose_3x2(Matrix3x2 matrix)
    {
        return Matrix2x3(
            matrix.m11, matrix.m21, matrix.m31,
            matrix.m12, matrix.m22, matrix.m32
        );
    }
    
    Matrix3x2 transpose_2x3(Matrix2x3 matrix)
    {
        return Matrix3x2(
            matrix.m11, matrix.m21,
            matrix.m12, matrix.m22,
            matrix.m13, matrix.m23
        );
    }

    Matrix2x2 inverse_2x2(Matrix2x2 matrix)
    {
        float coef = 1.0 / (matrix.m11*matrix.m22 - matrix.m12*matrix.m21);

        return Matrix2x2(
            matrix.m22*coef, -1.0*matrix.m21*coef,
            -1.0*matrix.m12*coef, matrix.m11*coef
        );
    }

    Matrix3x3 multiply(Matrix3x3 a, Matrix3x3 b)
    {
        return Matrix3x3(
            a.m11*b.m11 + a.m12*b.m21 + a.m13*b.m31, a.m11*b.m12 + a.m12*b.m22 + a.m13*b.m32, a.m11*b.m13 + a.m12*b.m23 + a.m13*b.m33,
            a.m21*b.m11 + a.m22*b.m21 + a.m23*b.m31, a.m21*b.m12 + a.m22*b.m22 + a.m23*b.m32, a.m21*b.m13 + a.m22*b.m23 + a.m23*b.m33,
            a.m31*b.m11 + a.m32*b.m21 + a.m33*b.m31, a.m31*b.m12 + a.m32*b.m22 + a.m33*b.m32, a.m31*b.m13 + a.m32*b.m23 + a.m33*b.m33);
    }

    Vector2 multiply(Matrix2x3 a, Vector3 b)
    {
        return Vector2(
            a.m11*b.x + a.m12*b.y + a.m13*b.z,
            a.m21*b.x + a.m22*b.y + a.m23*b.z);
    }

    Matrix3x2 multiply(Matrix3x3 a, Matrix3x2 b)
    {
        return Matrix3x2(
            a.m11*b.m11+a.m12*b.m21+a.m13*b.m31, a.m11*b.m12+a.m12*b.m22+a.m13*b.m32,
            a.m21*b.m11+a.m22*b.m21+a.m23*b.m31, a.m21*b.m12+a.m22*b.m22+a.m23*b.m32,
            a.m31*b.m11+a.m32*b.m21+a.m33*b.m31, a.m31*b.m12+a.m32*b.m22+a.m33*b.m32);
    }

    Vector3 multiply(Matrix3x2 a, Vector2 b)
    {
        return Vector3(
            a.m11*b.x + a.m12*b.y,
            a.m21*b.x + a.m22*b.y,
            a.m31*b.x + a.m32*b.y);
    } 

    Matrix3x2 multiply(Matrix3x2 a, Matrix2x2 b)
    {
        return Matrix3x2(
            a.m11*b.m11 + a.m12*b.m21, a.m11*b.m12 + a.m12*b.m22,
            a.m21*b.m11 + a.m22*b.m21, a.m21*b.m12 + a.m22*b.m22,
            a.m31*b.m11 + a.m32*b.m21, a.m31*b.m12 + a.m32*b.m22);
    }

    Matrix3x3 multiply(Matrix3x2 a, Matrix2x3 b)
    {
        return Matrix3x3(
            a.m11*b.m11+a.m12*b.m21, a.m11*b.m12+a.m12*b.m22, a.m11*b.m13+a.m12*b.m23,
            a.m21*b.m11+a.m22*b.m21, a.m21*b.m12+a.m22*b.m22, a.m21*b.m13+a.m22*b.m23,
            a.m31*b.m11+a.m32*b.m21, a.m31*b.m12+a.m32*b.m22, a.m31*b.m13+a.m32*b.m23);
    }

    Matrix2x2 to_2x2(Matrix3x3 matrix)
    {
        return Matrix2x2(
            matrix.m11, matrix.m12,
            matrix.m21, matrix.m22);
    }

    Matrix2x2 add(Matrix2x2 a, Matrix2x2 b)
    {
        return Matrix2x2(
            a.m11+b.m11, a.m12+b.m12,
            a.m21+b.m21, a.m22+b.m22);
    }

    Matrix3x3 add(Matrix3x3 a, Matrix3x3 b)
    {
        return Matrix3x3(
            a.m11+b.m11, a.m12+b.m12, a.m13+b.m13,
            a.m21+b.m21, a.m22+b.m22, a.m23+b.m23,
            a.m31+b.m31, a.m32+b.m32, a.m33+b.m33);
    }
}

#endif 