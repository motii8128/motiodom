#ifndef MOTISLAM_TYPES_HPP_
#define MOTISLAM_TYPES_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

namespace motiodom
{
    typedef Eigen::Vector2f Vec2;
    typedef Eigen::Vector3f Vec3;
    typedef Eigen::Matrix3f Mat3x3;
    typedef Eigen::Matrix2f Mat2x2;
    typedef Eigen::Matrix<float, 3, 2> Mat3x2;
    typedef Eigen::Matrix<float, 2, 3> Mat2x3;
    typedef Eigen::Quaternionf Quat;

    typedef std::vector<Vec2> PointCloud2d;

    /// @brief 度をラジアンに変換する
    /// @param deg 度[°]
    /// @return ラジアン[rad]
    float degree2radian(const float& deg);

    /// @brief オイラー角をクォータニオンに変換する
    /// @param euler オイラー角のベクトル
    /// @return クォータニオン
    Quat euler2quat(const Vec3& euler);
}

#endif