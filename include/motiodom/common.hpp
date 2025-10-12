#ifndef MOTIODOM_COMMON_HPP_
#define MOTIODOM_COMMON_HPP_

#include <nanoflann.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <vector>

namespace motiodom
{
    using Vec2 = Eigen::Vector2f;
    using Mat2 = Eigen::Matrix2f;
    using Mat3 = Eigen::Matrix3f;
    using Vec3 = Eigen::Vector3f;
    using Mat8 = Eigen::Matrix<float, 8, 8>;
    using Vec8 = Eigen::Matrix<float, 8, 1>;

    using PointCloud2f = std::vector<Vec2, Eigen::aligned_allocator<Vec2>>;

    struct PointCloudAdaptor {
        const PointCloud2f& pts;

        PointCloudAdaptor(const PointCloud2f& points)
            : pts(points) {}

        inline size_t kdtree_get_point_count() const {
            return pts.size();
        }

        inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
            return pts[idx][dim];
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const {
            return false;
        }
    };

    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor, 2>;
}

#endif