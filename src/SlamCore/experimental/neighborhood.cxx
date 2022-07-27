#include "SlamCore/experimental/neighborhood.h"

namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    NearestNeighborSearchResult::NearestNeighborSearchResult(int num_neighbors, double max_radius) {

        CHECK(num_neighbors > 0) << "The knn search only works for at least 1 neighbor" << std::endl;
        indices_.resize(num_neighbors);
        distances_.resize(num_neighbors);
        result_set = nanoflann::KNNRadiusResultSet(num_neighbors, max_radius);
        result_set.init(&indices_[0], &distances_[0]);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<size_t> NearestNeighborSearchResult::Indices() const {
        std::vector<size_t> result = indices_;
        result.resize(NumValidNeighbors());
        return result;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<double> NearestNeighborSearchResult::Distances() const {
        std::vector<double> result = distances_;
        result.resize(NumValidNeighbors());
        return result;
    }



} // namespace slam
