#ifndef SLAMCORE_NEIGHBORHOOD_H
#define SLAMCORE_NEIGHBORHOOD_H

#include <nanoflann.hpp>

#include "SlamCore/types.h"
#include "SlamCore/conversion.h"

namespace nanoflann {

    // A nanoflann result set to perform knn queries with additional radius limitation
    class KNNRadiusResultSet {
    public:
        typedef double DistanceType;
        typedef size_t IndexType;
        typedef size_t CountType;
    private:
        DistanceType max_radius_;
        IndexType *indices_ = nullptr;
        DistanceType *dists_ = nullptr;
        CountType capacity_;
        CountType count_;
    public:

        inline explicit KNNRadiusResultSet(CountType capacity_,
                                           double radius = std::numeric_limits<double>::max())
                : max_radius_(radius), capacity_(capacity_), count_(0) {}


        KNNRadiusResultSet() : KNNRadiusResultSet(1) {}

        inline void init(IndexType *indices, DistanceType *dists);

        inline CountType size() const { return count_; }

        inline bool full() const { return count_ == capacity_; }

        inline bool addPoint(DistanceType dist, IndexType index);

        inline DistanceType worstDist() const { return std::min(dists_[capacity_ - 1], max_radius_); }
    };

}

namespace slam {

    // A Simple wrapper for a NearestNeighborSearchResult
    class NearestNeighborSearchResult {
    public:

        explicit NearestNeighborSearchResult(int num_neighbors = 1,
                                             double max_radius = std::numeric_limits<double>::max());

        nanoflann::KNNRadiusResultSet &ResultSet() { return result_set; }

        inline int NumValidNeighbors() const { return result_set.size(); }

        std::vector<size_t> Indices() const;

        std::vector<double> Distances() const;

        inline const std::vector<size_t> &AllIndices() const { return indices_; };

        inline const std::vector<double> &AllDistances() const { return distances_; }

    private:
        std::vector<size_t> indices_;
        std::vector<double> distances_;
        nanoflann::KNNRadiusResultSet result_set;
    };


    // @brief A PointCloudAdaptor to build an index on a vector of Eigen::Vector3d using nanoflann
    //
    // Note: The PointCloudAdaptor does not create a copy of the point cloud, but uses a simple pointer
    //      Thus it is only valid as long as the pointer points to the correct data
    //
    template<typename _SourcePointT,
            typename _Conversion = slam::IdentityConversion<Eigen::Vector3d>,
            typename _Alloc = std::allocator<_SourcePointT>>
    class PointCloudAdaptor {
    public:
        PointCloudAdaptor() = default;

        explicit PointCloudAdaptor(const std::vector<_SourcePointT, _Alloc> *points_) : data(points_) {}

        inline size_t kdtree_get_point_count() const { return data->size(); };

        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            return conversion(data->at(idx))(static_cast<int>(dim));
        }

        template<class BBOX>
        bool kdtree_get_bbox(BBOX &) const { return false; }

        inline bool IsValid() const { return is_valid; }

        inline void SetData(const std::vector<_SourcePointT, _Alloc> *point_cloud) {
            static_assert(std::is_same_v<typename _Conversion::source_value_type, _SourcePointT>);
            static_assert(std::is_same_v<typename _Conversion::value_type, Eigen::Vector3d>);
            is_valid = point_cloud != nullptr;
            data = point_cloud;
        };

    private:
        _Conversion conversion;
        const std::vector<_SourcePointT, _Alloc> *data = nullptr;
        bool is_valid = false;
    };

    template<typename _SourcePointT,
            typename _Conversion = slam::IdentityConversion<Eigen::Vector3d>,
            typename _Alloc = std::allocator<_SourcePointT>>
    using neighborhood_kdtree_t = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor<_SourcePointT, _Conversion, _Alloc>>,
            PointCloudAdaptor<_SourcePointT, _Conversion, _Alloc>, 3>;
    typedef neighborhood_kdtree_t<Eigen::Vector3d> eigen_neighborhood_kdtree_t;

    typedef nanoflann::KNNRadiusResultSet result_set_t;

    // @brief NEIGHBORHOOD_PROPERTIES describe all properties of a NeighborhoodDescription
    enum NEIGHBORHOOD_PROPERTIES {
        NO_VALUES = 0,
        LINE = 1 << 0,
        NORMAL = 1 << 1,
        PLANARITY = 1 << 2,
        LINEARITY = 1 << 3,
        COV_DET = 1 << 4,
        KDTREE = 1 << 5,
        A2D = 1 << 6,
        ALL = LINE | NORMAL | PLANARITY | LINEARITY | COV_DET | KDTREE | A2D,
        ALL_BUT_KDTREE = LINE | NORMAL | PLANARITY | LINEARITY | COV_DET | A2D
    };

    /**
     * @brief Classification of the neighborhood
     */
    enum NEIGHBORHOOD_TYPE {
        NONE,
        LINEAR,
        PLANAR,
        VOLUMIC
    };

    // @brief Describes a neighborhood with a set of optional properties
    template<typename T>
    struct NeighborhoodDescription {

        T planarity = T(-1.0);

        T linearity = T(-1.0);

        T a2D = T(-1);

        T cov_det = T(-1.0); // Determinant of the covariance

        Eigen::Matrix<T, 3, 1> line;

        Eigen::Matrix<T, 3, 1> normal;

        Eigen::Matrix<T, 3, 1> barycenter;

        Eigen::Matrix<T, 3, 3> covariance;
    };

    // @brief Computes a Neighborhood Description from the covariance matrix of the spatial distribution of a set of points
    template<typename T>
    NeighborhoodDescription<T> ComputeNeighborhoodInfo(
            const Eigen::Vector3d &barycenter,
            const Eigen::Matrix<T, 3, 3> &covariance,
            int neighborhood_values = ALL);


    // @brief A Neighborhood is a set of points augmented with capacities to:
    //  - perform nearest neighbor queries
    //  - provide summaries of its spatial distribution.
    template<typename _SourcePointT = Eigen::Vector3d,
            typename _Conversion = slam::IdentityConversion<Eigen::Vector3d>,
            typename _Alloc = std::allocator<_SourcePointT>>
    struct TNeighborhood {
        typedef PointCloudAdaptor<_SourcePointT, _Conversion, _Alloc> pc_adaptor_t;
        typedef std::vector<_SourcePointT, _Alloc> vector_points_t;

        static int MinNeighborhoodSize() { return 5; };

        TNeighborhood() = default;

        explicit TNeighborhood(std::vector<_SourcePointT, _Alloc> &points);

        void ComputeNeighborhood(int values = int(ALL));

        bool SearchNearestNeighbors(const Eigen::Vector3d &query, result_set_t &result_set);

        void ClassifyNeighborhood(double planarity_threshold = 0.8,
                                  double linearity_threshold = 0.8);

        NeighborhoodDescription<double> description;
        vector_points_t points;
        int computed_values = NO_VALUES;
        NEIGHBORHOOD_TYPE neighborhood = NONE;
        pc_adaptor_t adaptor_;
        typedef neighborhood_kdtree_t<_SourcePointT, _Conversion, _Alloc> _neighborhood_kdtree_t;
        std::shared_ptr<_neighborhood_kdtree_t> index_ = nullptr;
        bool is_valid = false;
    };

    typedef TNeighborhood<Eigen::Vector3d> Neighborhood;
    typedef TNeighborhood<slam::WPoint3D, WorldPointConversion> WorldPointNeighborhood;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _SourcePointT, typename _Conversion, typename _Alloc>
    TNeighborhood<_SourcePointT, _Conversion, _Alloc>::TNeighborhood(std::vector<_SourcePointT, _Alloc> &points) :
            points(points) {
        static_assert(std::is_same_v<typename _Conversion::value_type, Eigen::Vector3d>);
        static_assert(std::is_same_v<typename _Conversion::source_value_type, _SourcePointT>);
        ComputeNeighborhood();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _SourcePointT, typename _Conversion, typename _Alloc>
    void TNeighborhood<_SourcePointT, _Conversion, _Alloc>::ComputeNeighborhood(int values) {
        if (points.size() < MinNeighborhoodSize()) {
            is_valid = false;
            return;
        }
        Eigen::Vector3d barycenter = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        _Conversion conversion;
        static_assert(std::is_same_v<typename _Conversion::value_type, Eigen::Vector3d>);

        Eigen::Vector3d point_ref;
        for (auto &point: points) {
            point_ref = conversion(point);
            barycenter += point_ref;
            cov += (point_ref * point_ref.transpose());
        }
        barycenter /= (double) points.size();
        cov /= (double) points.size();
        cov -= barycenter * barycenter.transpose();

        description = ComputeNeighborhoodInfo(barycenter, cov, values);
        computed_values = values;

        if (values & KDTREE) {
            adaptor_ = pc_adaptor_t(&points);
            index_ = std::make_shared<_neighborhood_kdtree_t>(3, adaptor_,
                                                              nanoflann::KDTreeSingleIndexAdaptorParams(10));
            index_->buildIndex();
        }
        is_valid = true;

    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _SourcePointT, typename _Conversion, typename _Alloc>
    bool TNeighborhood<_SourcePointT, _Conversion, _Alloc>::SearchNearestNeighbors(const Eigen::Vector3d &query,
                                                                                   result_set_t &result_set) {
        CHECK(index_) << "The Kdtree is not computed" << std::endl;
        return index_->findNeighbors(result_set, &query.x(), nanoflann::SearchParams(10));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _SourcePointT, typename _Conversion, typename _Alloc>
    void TNeighborhood<_SourcePointT, _Conversion, _Alloc>::ClassifyNeighborhood(double linearity_thresh,
                                                                                 double planarity_thresh) {
        if (computed_values & PLANARITY && computed_values & LINEARITY) {
            if (description.planarity > planarity_thresh) {
                neighborhood = PLANAR;
            } else if (description.linearity > linearity_thresh) {
                neighborhood = LINEAR;
            }
        } else if (computed_values & COV_DET && points.size() > 5) {
            neighborhood = VOLUMIC;
        } else {
            neighborhood = NONE;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    NeighborhoodDescription<T> ComputeNeighborhoodInfo(const Eigen::Vector3d &barycenter,
                                                       const Eigen::Matrix<T, 3, 3> &covariance, int values) {
        using Mat3 = Eigen::Matrix<T, 3, 3>;
        NeighborhoodDescription<T> result;
        result.covariance = covariance;
        result.barycenter = barycenter;

        Eigen::JacobiSVD<Mat3> svd(covariance, Eigen::ComputeFullV);

        if (values & LINE) {
            Mat3 V = svd.matrixV();
            result.line = V.template block<3, 1>(0, 0);
        }

        if (values & NORMAL) {
            Mat3 V = svd.matrixV();
            result.normal = V.template block<3, 1>(0, 2);
        }
        Eigen::Matrix<T, 3, 1> singular_values = svd.singularValues().cwiseAbs();
        if (values & LINEARITY)
            result.linearity = (singular_values[0] - singular_values[1]) / singular_values[0];
        if (values & PLANARITY)
            result.planarity = (singular_values[1] - singular_values[2]) / singular_values[0];
        if (values & A2D)
            result.a2D = (std::sqrt(singular_values[1]) - std::sqrt(singular_values[2])) / std::sqrt(
                    singular_values[0]); //Be careful, the eigenvalues are not correct with the iterative way to compute the covariance matrix
        if (values & COV_DET)
            result.cov_det = covariance.determinant();

        return result;
    }

}

namespace nanoflann {

    /* -------------------------------------------------------------------------------------------------------------- */
    void KNNRadiusResultSet::init(size_t *indices, double *dists) {
        indices_ = indices;
        dists_ = dists;
        count_ = 0;
        if (capacity_)
            dists_[capacity_ - 1] = (std::numeric_limits<DistanceType>::max)();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool KNNRadiusResultSet::addPoint(KNNRadiusResultSet::DistanceType dist, KNNRadiusResultSet::IndexType index) {
        // Skip over points at a distance over the max distance
        if (dist > max_radius_)
            return true;

        // Copied from nanoflann.hpp's KNNResultSet
        CountType i;
        for (i = count_; i > 0; --i) {
            if (dists_[i - 1] > dist) {
                if (i < capacity_) {
                    dists_[i] = dists_[i - 1];
                    indices_[i] = indices_[i - 1];
                }
            } else
                break;
        }
        if (i < capacity_) {
            dists_[i] = dist;
            indices_[i] = index;
        }
        if (count_ < capacity_)
            count_++;

        // tell caller that the search shall continue
        return true;
    }


} // namespace nanoflann

#endif //SLAMCORE_NEIGHBORHOOD_H
