#ifndef CT_ICP_SAMPLING_H
#define CT_ICP_SAMPLING_H

#include <vector>
#include <SlamCore/pointcloud.h>
#include <SlamCore/types.h>

namespace ct_icp {

    /**
     * Parameters for the AdaptiveGridSampling algorithm
     */
    struct AdaptiveGridSamplingOptions {
        int num_points_per_voxel = 1;   // Number of points kept per voxel
        int max_num_points = -1;         // Maximum number of points in total

        // Pairs distance to center - voxel size which defines the adaptive sample voxel size
        std::vector<std::pair<double, double>> distance_voxel_size = {
                {0.5,  0.1},
                {2.0,  0.2},
                {4.,   0.4},
                {8.,   0.8},
                {16.,  1.6},
                {200., -1.}
        };
    };

    /**
     * @brief Adaptive Sampling of a range of points
     *
     * @tparam IteratorT An type of iterator of Eigen::Vector3d
     * @returns The vector of selected indices
     */
    template<typename IteratorT>
    std::vector<size_t> AdaptiveSamplePointsInGrid(IteratorT begin,
                                                   IteratorT end,
                                                   const AdaptiveGridSamplingOptions &options);


    /**
     * @brief  Adaptive Sampling of a PointCloud
     * @return The PointCloud sampled (keeping all the fields of the original point cloud)
     */
    inline slam::PointCloudPtr AdaptiveSamplePointCloudInGrid(const slam::PointCloud &pc,
                                                              const AdaptiveGridSamplingOptions &options) {
        auto xyz = pc.XYZConst<double>();
        auto indices = AdaptiveSamplePointsInGrid(xyz.begin(), xyz.end(), options);
        return pc.SelectPoints(indices);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename IteratorT>
    std::vector<size_t>
    AdaptiveSamplePointsInGrid(IteratorT begin, IteratorT end, const AdaptiveGridSamplingOptions &options) {
        std::vector<std::unordered_map<slam::Voxel, std::vector<size_t>>> indices_maps(
                options.distance_voxel_size.size());
        const int kMaxNumPoints = options.max_num_points > 0 ? options.max_num_points : std::numeric_limits<int>::max();
        {
            auto current = begin;
            size_t idx(0);
            slam::Voxel voxel;
            while (current < end) {
                Eigen::Vector3d point = (*current);
                auto dist_to_lidar = point.norm();
                auto lw = std::lower_bound(options.distance_voxel_size.begin(),
                                           options.distance_voxel_size.end(),
                                           dist_to_lidar,
                                           [](const std::pair<double, double> &rhs, double lhs) {
                                               return rhs.first < lhs;
                                           });
                if (dist_to_lidar >= options.distance_voxel_size.front().first &&
                    dist_to_lidar < options.distance_voxel_size.back().first) {
                    auto _idx = std::distance(options.distance_voxel_size.begin(), lw) - 1;
                    auto voxel_size = options.distance_voxel_size[_idx].second;
                    voxel = slam::Voxel::Coordinates(point, voxel_size);

                    if (indices_maps[_idx].find(voxel) != indices_maps[_idx].end()) {
                        auto &indices = indices_maps[_idx][voxel];
                        if (indices.size() < options.num_points_per_voxel)
                            indices.push_back(idx);
                    } else
                        indices_maps[_idx][voxel].push_back(idx);
                }

                current++;
                idx++;
            }
        }

        std::vector<size_t> indices;
        for (const auto &_indices_map: indices_maps) {
            for (const auto &[_, _indices]: _indices_map) {
                for (auto idx: _indices) {
                    if (indices.size() > kMaxNumPoints)
                        break;
                    indices.push_back(idx);
                }

                if (indices.size() > kMaxNumPoints)
                    break;
            }

            if (indices.size() > kMaxNumPoints)
                break;
        }
        return indices;
    }

} // namespace ct_icp

#endif //CT_ICP_SAMPLING_H
