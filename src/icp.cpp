#include "motiodom/icp.hpp"

namespace motiodom
{
    ICP::ICP(int max_iter, float threshold) : max_iter_(max_iter), threshold_(threshold)
    {
        source_ = PointCloud2d();
    }

    void ICP::setSource(const PointCloud2d &pc)
    {
        source_ = pc;
    }

    Vec2 ICP::integrate(const PointCloud2d &pc, const float &yaw)
    {
        if(source_.empty() || pc.empty())
        {
            return Vec2();
        }

        Mat2x2 rotationMatrix;
        rotationMatrix << 
            cos(yaw), -1.0*sin(yaw),
            sin(yaw), cos(yaw);

        auto rotated_pc = PointCloud2d();
        for(const auto& p : pc)
        {
            const auto rotated_point = rotationMatrix * p;
            rotated_pc.push_back(rotated_point);
        }

            auto matched_target = PointCloud2d();
            for(const auto& src_p : source_)
            {
                auto min_dist = 10000.0;
                auto min_dist_p = Vec2();
                for(const auto& target_p : rotated_pc)
                {
                    const auto dx = src_p.x() - target_p.x();
                    const auto dy = src_p.y() - target_p.y();
                    const auto d = dx*dx + dy*dy;

                    if(d < min_dist)
                    {
                        min_dist = d;
                        min_dist_p = target_p;
                    }
                }

                matched_target.push_back(min_dist_p);
            }

            Vec2 translation;

            for(int i = 0; i < source_.size(); i++)
            {
                const auto src_p = source_[i];
                const auto tar_p = matched_target[i];

                translation += src_p - tar_p;
            }

            translation /= source_.size();

        return translation;
    }
}