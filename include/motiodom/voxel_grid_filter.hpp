#ifndef MOTIODOM_VOXEL_GRID_FILTER_HPP_
#define MOTIODOM_VOXEL_GRID_FILTER_HPP_

#include "common.hpp"
#include <unordered_map>
#include <cmath>

namespace motiodom
{
    struct VoxelKey2D
    {
        int ix, iy;
        bool operator==(const VoxelKey2D& other) const noexcept {
            return ix == other.ix && iy == other.iy;
        }
    };

    struct VoxelKey2DHash
    {
        std::size_t operator()(const VoxelKey2D& key) const noexcept {
            // シンプルかつ衝突を減らす整数ハッシュ
            return static_cast<std::size_t>(
                (key.ix * 73856093) ^ (key.iy * 19349663)
            );
        }
    };

    /// @brief ボクセルグリッドフィルタによるダウンサンプリング
    /// @param input 入力点群
    /// @param voxel_size グリッドサイズ
    /// @return フィルタ後点群
    PointCloud2f voxelGridFilter2D(const PointCloud2f& input, float voxel_size);
}

#endif