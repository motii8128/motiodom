#include "motiodom/voxel_grid_filter.hpp"

namespace motiodom
{
    PointCloud2f voxelGridFilter2D(const PointCloud2f& input, float voxel_size)
    {
        using VoxelMap = std::unordered_map<VoxelKey2D, PointCloud2f, VoxelKey2DHash>;
        VoxelMap voxel_map;

        // 各点をボクセルに分類
        for (const auto& p : input)
        {
            int ix = static_cast<int>(std::floor(p.x() / voxel_size));
            int iy = static_cast<int>(std::floor(p.y() / voxel_size));

            VoxelKey2D key{ix, iy};
            voxel_map[key].push_back(p);
        }

        // 各ボクセルの重心を出力点として計算
        PointCloud2f output;
        output.reserve(voxel_map.size());

        for (const auto& kv : voxel_map)
        {
            const auto& pts = kv.second;
            Point2f mean = Point2f::Zero();
            for (const auto& p : pts)
                mean += p;
            mean /= static_cast<float>(pts.size());
            output.push_back(mean);
        }

        return output;
    }
}