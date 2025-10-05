#ifndef MOTIODOM_COMMON_HPP_
#define MOTIODOM_COMMON_HPP_

#include <nanoflann.hpp>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <vector>

namespace motiodom
{
    using Point2f = Eigen::Vector2f;
    using Point3f = Eigen::Vector3f;
    using PointCloud2f = std::vector<Point2f, Eigen::aligned_allocator<Point2f>>;
    using Mat2 = Eigen::Matrix2f;

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