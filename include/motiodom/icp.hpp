#ifndef ICP_SLAM_HPP_
#define ICP_SLAM_HPP_

#include "types.hpp"

namespace motiodom
{
    class ICP
    {
        public:
        /// @brief　コンストラクタ
        /// @param max_iter 最大実行回数
        /// @param threshold 許容誤差
        ICP(int max_iter=10, float threshold=0.01);

        void setSource(const PointCloud2d &pc);
        Vec2 integrate(const PointCloud2d &pc, const float &yaw);

        private:
        PointCloud2d source_;
        int max_iter_;
        float threshold_;
    };
}

#endif