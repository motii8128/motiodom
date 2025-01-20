#ifndef ICP_SLAM_HPP_
#define ICP_SLAM_HPP_

#include "types.hpp"

namespace motiodom
{
    class ICP_SLAM
    {
        public:
        /// @brief　コンストラクタ
        /// @param max_iter 最大実行回数
        /// @param threshold 許容誤差
        ICP_SLAM(int max_iter, float threshold);

        void setSource(const PointCloud2d pc);
        void integrate(const PointCloud2d pc, const Quat posture);

        private:
        PointCloud2d source_;
        int max_iter_;
        float threshold_;
    };
}

#endif