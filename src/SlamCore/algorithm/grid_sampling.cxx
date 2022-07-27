#include "SlamCore/algorithm/grid_sampling.h"

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::PointCloudPtr SamplePointCloudInGrid(const PointCloud &pc, const GridSamplingOptions &options) {
        auto xyz = pc.XYZConst<double>();
        auto indices = SamplePointsInGrid(xyz.begin(), xyz.end(), options);
        return pc.SelectPoints(indices);
    }

} // namespace slam