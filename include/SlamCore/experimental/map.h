#ifndef SLAMCORE_MAP_H
#define SLAMCORE_MAP_H

#include <vector>
#include <Eigen/Dense>
#include <queue>

#include <tsl/robin_map.h>

#include "SlamCore/conversion.h"
#include "SlamCore/pointcloud.h"
#include "SlamCore/experimental/neighborhood.h"

namespace slam {

    /*!
     * @brief   An Interface for an abstract map
     */
    class IMap {
    public:

        virtual ~IMap() = 0;

        // @brief   Clears the map
        virtual void ClearMap() = 0;

        // @brief   Adds a Point Cloud to the map
        virtual void InsertPointCloud(const slam::PointCloud &,
                                      std::vector<size_t> &out_selected_points) = 0;

        // @brief   Removes elements of the map far from the given location
        virtual void RemoveElementsFarFromLocation(const Eigen::Vector3d &location, double distance) = 0;

        // @brief   Returns a Neighborhood for a given spatial query
        Neighborhood ComputeNeighborhood(const Eigen::Vector3d &query, int max_num_neighbors) const;

        /**
         * @brief  Constructs a neighborhood in place for a given spatial query
         */
        virtual void ComputeNeighborhoodInPlace(const Eigen::Vector3d &query, int max_num_neighbors,
                                                Neighborhood &neighborhood) const = 0;

        // @brief   Returns a vector of neighborhood from a vector of queries
        virtual std::vector<Neighborhood> ComputeNeighborhoods(const std::vector<Eigen::Vector3d> &queries,
                                                               int max_num_neighbors) const = 0;
    };


    /*!
     * @brief   An AVoxelBlock is the data block contained Voxel Hash Map
     *
     * @tparam _Derived     The Derived Class, implementing the CRTP pattern
     * @tparam _PointT      The Point Type, which must match the _Derived point_type
     * @tparam _Iterator
     */
    template<typename _Derived, typename _PointT, typename _Iterator>
    class AVoxelBlock {
    public:
        typedef _PointT point_type;

#define AVoxelBlock_static_check static_assert(std::is_same_v<_Iterator, typename _Derived::iterator>); \
                                 static_assert(std::is_same_v<typename _Iterator::iterator_category,    \
                                                              std::random_access_iterator_tag>);

#define AVoxelBlock_THIS  static_cast<_Derived*>(this)
#define AVoxelBlock_CONST_THIS  static_cast<const _Derived*>(this)

        AVoxelBlock() { AVoxelBlock_static_check }

        inline void insert(const _PointT &point) {
            AVoxelBlock_static_check
            AVoxelBlock_THIS->__insert(point);
        };

        inline void reserve(size_t __n) {
            AVoxelBlock_static_check
            return AVoxelBlock_THIS->__reserve(__n);
        }

        inline _Iterator begin() {
            AVoxelBlock_static_check
            return AVoxelBlock_THIS->__begin();
        }

        inline _Iterator end() {
            AVoxelBlock_static_check
            return AVoxelBlock_THIS->__end();
        }

        inline _Iterator cbegin() const {
            AVoxelBlock_static_check
            return AVoxelBlock_CONST_THIS->__cbegin();
        }

        inline _Iterator cend() const {
            AVoxelBlock_static_check
            return AVoxelBlock_CONST_THIS->__cend();
        }

        inline _PointT &operator[](size_t __n) { return begin() + __n; }

        inline const _PointT &operator[](size_t __n) const { return begin() + __n; }

        inline size_t size() const { return AVoxelBlock_CONST_THIS->__size(); }

    };

    /*!
     * @brief   A simple Vector Based Voxel block
     */
    template<typename _PointT, typename _Alloc = std::allocator<_PointT>>
    struct VectorVBlock : AVoxelBlock<VectorVBlock<_PointT, _Alloc>, _PointT,
            typename std::vector<_PointT, _Alloc>::iterator> {

        typedef AVoxelBlock<VectorVBlock<_PointT, _Alloc>, _PointT,
                typename std::vector<_PointT, _Alloc>::iterator> __parent_t;
        typedef typename std::vector<_PointT, _Alloc>::iterator iterator;
        using typename __parent_t::point_type;

        inline void __reserve(size_t __n) { points.reserve(__n); }

        inline void __insert(const _PointT &point) { points.push_back(point); }

        inline iterator __begin() { return points.begin(); }

        inline iterator __cbegin() const { return points.cbegin(); }

        inline iterator __end() { return points.end(); }

        inline iterator __cend() const { return points.cend(); }

        inline size_t __size() const { return points.size(); }

        std::vector<_PointT, _Alloc> points;
    };


    /*!
     * @brief   A standard Voxel HashMap which allows no repetition
     *
     * @tparam  _VoxelBlockT    The type of a Voxel block
     * @tparam  _ConversionT    The conversion from a _PointT
     * @tparam  _PointT         The type of points expected
     */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    class VoxelHashMap : public IMap {
    public:
        typedef VoxelHashMap<_VoxelBlockT, _PointT, _ConversionT> this_type;
        typedef tsl::robin_map<Voxel, _VoxelBlockT> hash_map_t;
        typedef typename hash_map_t::iterator iterator;
        typedef typename hash_map_t::const_iterator const_iterator;

        struct TopologyOptions {
            double voxel_size = 1.0;                    // The size of a voxel in the 3D Space
            int max_voxel_block_size = 20;              // The maximum number of points to put in a voxel map
            double min_distance_between_points = 0.10;  // The minimum distance between two inserted points
        };

        struct SearchOptions {
            double max_radius = 1.0;        // The maximum search radius during a neighborhood search
            int voxel_radius = 1;           // The number of voxels to iterate during a neighborhood search
        };

        ~VoxelHashMap() = default;

        explicit VoxelHashMap(const TopologyOptions &options) : options_(options) {}


        VoxelHashMap(const TopologyOptions &options,
                     const SearchOptions &search_options) : options_(options),
                                                            search_options_(search_options) {}

        VoxelHashMap() = default;

        REF_GETTER(GetSearchOptions, search_options_);

        template<typename __PointIterator>
        std::vector<size_t> InsertPoints(__PointIterator begin, __PointIterator end);

        inline bool InsertPoint(const slam::Voxel &voxel, const _PointT &point);

        inline size_t size() {
            return hash_map_.size();
        }

        inline iterator begin() {
            return hash_map_.begin();
        }

        inline const_iterator begin() const {
            return hash_map_.cbegin();
        }

        inline iterator end() {
            return hash_map_.end();
        }

        inline const_iterator end() const {
            return hash_map_.cend();
        }

        inline slam::Voxel GetVoxel(const Eigen::Vector3d &location) const;

        inline bool HasVoxel(const slam::Voxel &voxel) const;

        _VoxelBlockT &operator[](const slam::Voxel &voxel);

        const _VoxelBlockT &at(const slam::Voxel &voxel) const;

        /// --- IMap API ---

        void ClearMap() override;

        // @brief   Adds a Point Cloud to the map
        void InsertPointCloud(const slam::PointCloud &cloud,
                              std::vector<size_t> &out_selected_points) override {
            auto &collection = cloud.GetCollection();
            CHECK(collection.GetItemInfo(0).item_size == sizeof(_PointT))
                            << "The point cloud does no have a compatible item size in its first item buffer";
            auto item_view = collection.template item<_PointT>(0);
            out_selected_points = InsertPoints(item_view.cbegin(), item_view.cend());
        };

        // @brief   Removes elements of the map far from the given location
        void RemoveElementsFarFromLocation(const Eigen::Vector3d &location, double distance) override;

        // @brief   Returns a Neighborhood for a given spatial query
        inline void __ComputeNeighborhood(const Eigen::Vector3d &query, int max_num_neighbors, Neighborhood &) const;


        // @brief   Returns a Neighborhood for a given spatial query
        inline void ComputeNeighborhoodInPlace(const Eigen::Vector3d &query,
                                               int max_num_neighbors,
                                               Neighborhood &neighborhood) const override {
            return __ComputeNeighborhood(query, max_num_neighbors, neighborhood);
        }

        // @brief   Returns a vector of neighborhood from a vector of queries
        std::vector<Neighborhood> ComputeNeighborhoods(const std::vector<Eigen::Vector3d> &queries,
                                                       int max_num_neighbors) const override {
            std::vector<Neighborhood> neighborhoods(queries.size());
            for (auto i(0); i < queries.size(); ++i)
                __ComputeNeighborhood(queries[i], max_num_neighbors, neighborhoods[i]);
            return neighborhoods;
        };

    private:
        TopologyOptions options_;
        SearchOptions search_options_;
        _ConversionT conversion_;
        tsl::robin_map<Voxel, _VoxelBlockT> hash_map_;

        using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;

        struct __Comparator {
            bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
                return std::get<0>(left) < std::get<0>(right);
            }
        };

        typedef std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, __Comparator> priority_queue_t;
    };

    // -- Voxel map Specifications
    typedef VoxelHashMap<VectorVBlock<Eigen::Vector3f>,
            Eigen::Vector3f, EigenCopyConversion<double, float, 3, 1>> VoxelHashMapVec3f;
    typedef VoxelHashMap<VectorVBlock<Eigen::Vector3d>,
            Eigen::Vector3d, IdentityConversion<Eigen::Vector3d>> VoxelHashMapVec3d;


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    template<typename __PointIterator>
    std::vector<size_t>
    VoxelHashMap<_VoxelBlockT, _PointT, _ConversionT>::InsertPoints(__PointIterator begin, __PointIterator end) {
        static_assert(std::is_same_v<typename __PointIterator::value_type, typename _VoxelBlockT::point_type>);
        const double voxel_size = options_.voxel_size;
        std::vector<size_t> inserted_ids;
        _PointT new_point;
        slam::Voxel voxel;
        size_t _id(0);
        while (begin != end) {
            new_point = *begin;
            voxel = slam::Voxel::Coordinates(conversion_(new_point), voxel_size);
            if (InsertPoint(voxel, new_point))
                inserted_ids.push_back(_id);
            begin++;
            _id++;
        }
        return inserted_ids;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    bool VoxelHashMap<_VoxelBlockT, _PointT, _ConversionT>::InsertPoint(const Voxel &voxel, const _PointT &point) {
        if (hash_map_.find(voxel) == hash_map_.end()) {
            hash_map_[voxel].reserve(options_.max_voxel_block_size);
            hash_map_[voxel].insert(point);
            return true;
        }
        auto &voxel_block = hash_map_[voxel];
        if (voxel_block.size() < options_.max_voxel_block_size) {
            double sq_dist_min_to_points = std::numeric_limits<double>::max();
            // Compute the min distance between the
            for (int i(0); i < voxel_block.size(); ++i) {
                auto &_point = voxel_block.points[i];
                double sq_dist = (_point - point).squaredNorm();
                if (sq_dist < sq_dist_min_to_points) {
                    sq_dist_min_to_points = sq_dist;
                }
            }
            if (sq_dist_min_to_points > (options_.min_distance_between_points *
                                         options_.min_distance_between_points)) {
                voxel_block.insert(point);
                return true;
            }
        }
        return false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    void VoxelHashMap<_VoxelBlockT, _PointT, _ConversionT>::ClearMap() {
        hash_map_.clear();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    void VoxelHashMap<_VoxelBlockT,
            _PointT,
            _ConversionT>::__ComputeNeighborhood(const Eigen::Vector3d &query,
                                                 int max_num_neighbors, Neighborhood &neighborhood) const {
        const double voxel_size = options_.voxel_size;
        const int nb_voxels_visited = search_options_.voxel_radius;
        const double max_neighborhood_radius = search_options_.max_radius;
        Voxel voxel = Voxel::Coordinates(query, voxel_size);
        int kx = voxel.x;
        int ky = voxel.y;
        int kz = voxel.z;

        Eigen::Vector3d neighbor;
        priority_queue_t priority_queue;
        for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
            for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
                for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                    voxel.x = kxx;
                    voxel.y = kyy;
                    voxel.z = kzz;

                    auto search = hash_map_.find(voxel);
                    if (search != hash_map_.end()) {
                        const auto &voxel_block = search.value();
                        for (int i(0); i < voxel_block.size(); ++i) {
                            neighbor = conversion_(voxel_block.points[i]);
                            double distance = (neighbor - query).norm();
                            if (distance > max_neighborhood_radius)
                                continue;
                            if (priority_queue.size() == max_num_neighbors) {
                                if (distance < std::get<0>(priority_queue.top())) {
                                    priority_queue.pop();
                                    priority_queue.emplace(distance, neighbor, voxel);
                                }
                            } else
                                priority_queue.emplace(distance, neighbor, voxel);
                        }
                    }
                }
            }
        }

        neighborhood.points.resize(0);
        neighborhood.points.reserve(priority_queue.size());
        while (!priority_queue.empty()) {
            neighborhood.points.push_back(std::get<1>(priority_queue.top()));
            priority_queue.pop();
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    void VoxelHashMap<_VoxelBlockT,
            _PointT, _ConversionT>::RemoveElementsFarFromLocation(const Eigen::Vector3d &location,
                                                                  double distance) {
        const double voxel_size = options_.voxel_size;
        std::set<Voxel> voxels_to_erase;
        Eigen::Vector3d voxel_center;
        for (auto &voxel: *this) {
            voxel_center = Eigen::Vector3d((voxel.first.x + 0.5) * voxel_size,
                                           (voxel.first.y + 0.5) * voxel_size,
                                           (voxel.first.z + 0.5) * voxel_size);
            if ((location - voxel_center).norm() > distance)
                voxels_to_erase.emplace(voxel.first);
        }
        for (auto &voxel: voxels_to_erase)
            hash_map_.erase(voxel);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    _VoxelBlockT &VoxelHashMap<_VoxelBlockT, _PointT, _ConversionT>::operator[](const Voxel &voxel) {
        return hash_map_[voxel];
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    bool VoxelHashMap<_VoxelBlockT, _PointT, _ConversionT>::HasVoxel(const Voxel &voxel) const {
        return hash_map_.contains(voxel);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    const _VoxelBlockT &VoxelHashMap<_VoxelBlockT, _PointT, _ConversionT>::at(const Voxel &voxel) const {
        return hash_map_.at(voxel);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename _VoxelBlockT, class _PointT, class _ConversionT>
    slam::Voxel VoxelHashMap<_VoxelBlockT, _PointT, _ConversionT>::GetVoxel(const Eigen::Vector3d &location) const {
        return slam::Voxel::Coordinates(location, options_.voxel_size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */

} // namespace slam

#endif //SLAMCORE_MAP_H
