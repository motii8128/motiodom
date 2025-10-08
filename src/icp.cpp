#include "motiodom/icp.hpp"

namespace motiodom
{
    ICP2D::ICP2D(int max_iter, float tolerance_trans, float tolerance_rot, float max_corr_dist):
    max_iter_(max_iter),
    tolerance_trans_(tolerance_trans),
    tolerance_rot_(tolerance_rot),
    max_corr_dist_(max_corr_dist)
    {

    }

    ICPResult ICP2D::align(PointCloud2f& map, const PointCloud2f& current, Mat2& R, Point2f& t)
    {
        auto source = voxelGridFilter2D(current, 0.05);
        for(auto& p : source)
        {
            p = R * p + t;
        }

        // KDTree構築
        PointCloudAdaptor target_adaptor(map);
        KDTree kdtree(2, target_adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree.buildIndex();

        int iter_num = 0;
        bool has_cov = false;

        for(int i = 0; i < max_iter_; i++)
        {
            std::vector<int> neighbors = find_neighbor(source, kdtree, max_corr_dist_);

            PointCloud2f src_corr, tgt_corr;
            src_corr.reserve(source.size());
            tgt_corr.reserve(source.size());
            for (size_t i = 0; i < neighbors.size(); ++i) 
            {
                int idx = neighbors[i];
                if(idx >= 0) 
                {
                    src_corr.push_back(source[i]);
                    tgt_corr.push_back(map[idx]);
                }
            }

            if(src_corr.size() < 3)
            {
                ICPResult result;
                result.has_covered = false;
                result.iter = -1;
                return result;
            }

            Eigen::Matrix2f dR;
            Eigen::Vector2f dt;
            estimateRigidTransform(src_corr, tgt_corr, dR, dt);

            for(auto& p : source)
            {
                p = dR * p + dt;
            }

            R = dR * R;
            t = dR * t + dt;
            float delta_trans = dt.norm();
            float delta_rot = std::fabs(std::atan2(dR(1, 0), dR(0, 0)));
            if (delta_trans < tolerance_trans_ && delta_rot < tolerance_rot_) {
                has_cov = true;
                break;
            }

            iter_num++;
        }

        ICPResult result;
        result.iter = iter_num;
        result.has_covered = has_cov;

        if(has_cov)
        {
            PointCloud2f transformed = source;
            map.insert(map.end(), transformed.begin(), transformed.end());
            map = voxelGridFilter2D(map, 0.05);
        }

        return result;
    }

    void ICP2D::estimateRigidTransform(const PointCloud2f& src, const PointCloud2f& target, Mat2& R, Point2f& t)
    {
        int N = src.size();

        Point2f mu_src = Point2f::Zero();
        Point2f mu_target = Point2f::Zero();

        for(int i = 0; i < N; i++)
        {
            mu_src += src[i];
            mu_target += target[i];
        }
        mu_src /= N;
        mu_target /= N;

        Eigen::MatrixXf src_centered(2, N), target_centered(2, N);
        for (int i = 0; i < N; ++i) {
            src_centered.col(i) = src[i] - mu_src;
            target_centered.col(i) = target[i] - mu_target;
        }

        Mat2 W = src_centered * target_centered.transpose();
        Eigen::JacobiSVD<Mat2> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Mat2 U = svd.matrixU();
        Mat2 V = svd.matrixV();

        R = V * U.transpose();
        if(R.determinant() < 0)
        {
            V.col(1) *= -1;
            R = V * U.transpose();
        }

        t = mu_target - R * mu_src;
    }

    std::vector<int> find_neighbor(const PointCloud2f& src, KDTree& kdtree, float max_dist)
    {
        std::vector<int> neighbor(src.size(), -1);

        for(size_t i = 0; i < src.size(); i++)
        {
            const float query_pt[2] = {src[i].x(), src[i].y()};

            size_t nearest_index;
            float out_dist_sqr;

            nanoflann::KNNResultSet<float> result_set(1);
            result_set.init(&nearest_index, &out_dist_sqr);
            kdtree.findNeighbors(result_set, query_pt, nanoflann::SearchParameters(10));

            if(out_dist_sqr < max_dist * max_dist)
            {
                neighbor[i] = static_cast<int>(nearest_index);
            }
        }

        return neighbor;
    }
}