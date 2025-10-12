#ifndef MOTIODOM_ICP_HPP_
#define MOTIODOM_ICP_HPP_

#include "common.hpp"
#include "voxel_grid_filter.hpp"

namespace motiodom
{
    /// @brief ICPの結果を格納する
    struct ICPResult
    {
        bool has_covered;
        int iter;
    };
    /// @brief ２次元InteractiveClosestPointを実装したクラス
    class ICP2D
    {
        public:
        /// @brief コンストラクタ
        /// @param max_iter 最大実行回数
        /// @param tolerance_trans 位置の収束閾値
        /// @param tolerance_rot 回転の収束閾値
        /// @param max_corr_dist 最近傍探索時の最長距離
        ICP2D(int max_iter = 50, float tolerance_trans = 1e-4f, float tolerance_rot = 1e-4f, float max_corr_dist = 1.0f);

        /// @brief ICPによる点群の位置合わせを実行する
        /// @param map マップ点群
        /// @param current 現在スキャン点群
        /// @param R_out 回転(初期推定)
        /// @param t_out 変位(初期推定)
        /// @return ICPの結果（収束したか、実行回数）
        ICPResult align(PointCloud2f& map, const PointCloud2f& current, Mat2& R, Vec2& t);

        private:
        /// @brief ペアリング済みの点群から剛体変換を求める
        /// @param src 参照点群
        /// @param target 目標点群
        /// @param R 回転行列
        /// @param t 変位
        void estimateRigidTransform(const PointCloud2f& src, const PointCloud2f& target, Mat2& R, Vec2& t);

        int max_iter_;
        float tolerance_trans_;
        float tolerance_rot_;
        float max_corr_dist_;
    };

    /// @brief 最近傍探索
    /// @param src 参照点群
    /// @param kdtree マップ点群のKDTree
    /// @param max_dist 最長距離
    /// @return 最近傍インデックス
    std::vector<int> find_neighbor(const PointCloud2f& src, KDTree& kdtree, float max_dist = 1.0f);    
}

#endif