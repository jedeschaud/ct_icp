#include "SlamCore/experimental/map.h"

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    IMap::~IMap() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    Neighborhood IMap::ComputeNeighborhood(const Eigen::Vector3d &query, int max_num_neighbors) const {
        Neighborhood neighborhood;
        ComputeNeighborhoodInPlace(query, max_num_neighbors, neighborhood);
        return neighborhood;
    }
} // namespace slam