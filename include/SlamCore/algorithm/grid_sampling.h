#ifndef SLAMCORE_GRID_SAMPLING_H
#define SLAMCORE_GRID_SAMPLING_H

#include <vector>
#include <unordered_map>

#include "SlamCore/types.h"
#include "SlamCore/pointcloud.h"

namespace slam {

    struct GridSamplingOptions {
        double grid_size = 1.0;
        int num_points_per_voxel = 1;
        int max_num_points = -1;
    };

    /**
     * @brief A Grid Sampling algorithm
     *
     * @returns The indices of the points sampled in the grid
     */
    template<typename IteratorT>
    std::vector<size_t> SamplePointsInGrid(IteratorT begin, IteratorT end,
                                           const GridSamplingOptions &options);

    /**
     * @brief Sample points from a point cloud using a grid sampling algorithm
     */
    slam::PointCloudPtr SamplePointCloudInGrid(const slam::PointCloud &pc, const GridSamplingOptions &options);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename IteratorT>
    std::vector<size_t> SamplePointsInGrid(IteratorT begin, IteratorT end, const GridSamplingOptions &options) {
        const double kVoxelSize = options.grid_size;
        const size_t kMaxNumPoints = options.max_num_points < 0 ?
                                     std::numeric_limits<size_t>::max() : size_t(options.max_num_points);
        std::unordered_map<slam::Voxel, std::vector<size_t>> map_of_indices;
        {
            auto current = begin;
            size_t idx(0);
            slam::Voxel voxel;
            while (current < end) {
                voxel = slam::Voxel::Coordinates(*current, kVoxelSize);
                if (map_of_indices.find(voxel) != map_of_indices.end()) {
                    auto &indices = map_of_indices[voxel];
                    if (indices.size() < options.num_points_per_voxel)
                        indices.push_back(idx);
                } else
                    map_of_indices[voxel].push_back(idx);

                current++;
                idx++;
            }
        }

        std::vector<size_t> indices;
        for (const auto &[_, _indices]: map_of_indices) {
            for (auto idx: _indices) {
                if (indices.size() > kMaxNumPoints)
                    break;
                indices.push_back(idx);
            }
            if (indices.size() > kMaxNumPoints)
                break;
        }
        return indices;
    }


} // namespace slam

#endif //SLAMCORE_GRID_SAMPLING_H