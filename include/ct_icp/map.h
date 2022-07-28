#ifndef CT_ICP_MAP_H
#define CT_ICP_MAP_H

#include <SlamCore/conversion.h>
#include <SlamCore/experimental/map.h>
#include <SlamCore/trajectory.h>
#include <SlamCore/types.h>
#include <SlamCore/config_utils.h>

namespace ct_icp {

    /*! @brief Abstract map interface
     */
    class ISlamMap : public slam::IMap {
    public:

        /////////////////////////////////////////
        /// Insertions
        /////////////////////////////////////////

        /*!
         * @brief Inserts a point cloud, providing the poses the point cloud was built from
         *
         */
        virtual void InsertPointCloud(const slam::PointCloud &pointcloud,
                                      const std::vector<slam::Pose> &frame_poses,
                                      std::vector<size_t> &out_indices) = 0;

        using slam::IMap::InsertPointCloud;

        /////////////////////////////////////////
        /// Exporting the map
        /////////////////////////////////////////

        /*!
         * @brief Returns the number of points in the map
         */
        virtual size_t NumPoints() const = 0;

        /*!
         * @brief Extract the map as a point cloud
         */
        virtual slam::PointCloudPtr MapAsPointCloud() const = 0;

        /////////////////////////////////////////
        /// Update trajectory
        /////////////////////////////////////////

        virtual void UpdateTrajectory(const std::vector<slam::Pose> &poses) {};

        virtual void UpdateTrajectory(const slam::LinearContinuousTrajectory &trajectory) {};

        /////////////////////////////////////////
        /// Random neighborhood
        /////////////////////////////////////////

        /*! @brief   Returns a Neighborhood for a given spatial query
         *
         * @param nearest_neighbors Whether to use nearest neighbors for the neighborhood computation (or a random
         *                          neighbors in the radius neighborhood)
         */
        virtual slam::Neighborhood RadiusSearch(const Eigen::Vector3d &query,
                                                double radius, int max_num_neighbors = -1,
                                                bool nearest_neighbors = true,
                                                Eigen::Vector3d *sensor_location = nullptr) const = 0;

        /*!
         * @brief  Constructs a neighborhood in place for a given spatial query
         *
         * @param sensor_location Optional location of the sensor (allows to filter out points with incompatible normals)
         */
        virtual void RadiusSearchInPlace(const Eigen::Vector3d &query, slam::Neighborhood &neighborhood,
                                         double radius, int max_num_neighbors = -1,
                                         bool nearest_neighbors = true,
                                         Eigen::Vector3d *sensor_location = nullptr) const = 0;

        // @brief   Returns a vector of neighborhood from a vector of queries
        virtual std::vector<slam::Neighborhood> ComputeNeighborhoods(const std::vector<Eigen::Vector3d> &queries,
                                                                     const std::vector<double> radiuses,
                                                                     int max_num_neighbors,
                                                                     bool nearest_neighbors,
                                                                     Eigen::Vector3d *sensor_location) const = 0;
    };

    struct IMapOptions {

        virtual ~IMapOptions() = 0;

        virtual std::string GetType() const { return "INVALID_MAP"; }

        virtual std::shared_ptr<ct_icp::ISlamMap> MakeMapFromOptions() const {
            throw std::runtime_error("Not implemented Error");
        };
    };

    /*!
     * @brief A MultipleResolutionVoxelMap which stores multiple voxel maps at different resolutions
     */
    class MultipleResolutionVoxelMap : public ISlamMap {
    public:

        struct SearchParams {
            double radius = 0.5;
            double voxel_resolution = 0.;
            size_t map_id = 0;
            int voxel_neighborhood = 1;
        };

        struct ResolutionParam {
            double resolution = 0.5;
            double min_distance_between_points = 0.1;
            int max_num_points = 40;
        };

        struct Options : public IMapOptions {

            std::vector<ResolutionParam> resolutions = {
                    ResolutionParam{0.2, 0.03, 50},
                    ResolutionParam{0.5, 0.1, 40},
                    ResolutionParam{1.5, 0.15, 40}
            };

            bool select_valid_normals_direction = true; //< Use the normals direction to filter inconsistent points (behind the normal plane)
            size_t max_frames_to_keep = 100; //< The number of frames to keep in the map
            double default_radius = 0.8; //< The default radius for search with uniform radius

            static std::string Type() { return "MULTI_RESOLUTION_VOXEL_HASHMAP"; }

            std::string GetType() const override { return Type(); }

            inline std::shared_ptr<ISlamMap> MakeMapFromOptions() const final {
                return std::make_shared<MultipleResolutionVoxelMap>(*this);
            };
        };


        explicit MultipleResolutionVoxelMap(const Options &options) : options_(options) {
            voxel_maps_.resize(options.resolutions.size());
        }

        MultipleResolutionVoxelMap() : MultipleResolutionVoxelMap(Options()) {}

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// UPDATE API
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /*!
         * @brief Inserts a the points of a point cloud in the map.
         *
         * The points of the world coordinate frame will be inserted in the map.
         * If only the Raw Points field in the Point Cloud is defined, the points will be transformed using the provided poses
         */
        void InsertPointCloud(const slam::PointCloud &pointcloud,
                              const std::vector<slam::Pose> &frame_poses,
                              std::vector<size_t> &out_indices) override {
            SLAM_CHECK_STREAM(!frame_poses.empty(), "the poses are empty");
            auto fidx = frame_id_count_++;
            frame_id_to_frame[fidx] = {pointcloud.DeepCopyPtr(),
                                       slam::LinearContinuousTrajectory::Create(std::vector<slam::Pose>(frame_poses))};
            auto &frame = frame_id_to_frame[fidx];
            auto &pc = frame.pointcloud;
            auto &trajectory = frame.poses;

            pc->RegisterFieldsFromSchema();
            if (!pc->HasWorldPoints()) {
                // Construct the WorldPointsField in the point cloud
                SLAM_CHECK_STREAM(pc->HasRawPoints(), "The input point cloud does not have raw points defined");
                pc->AddDefaultWorldPointsField();

                // Transform the raw points using the poses
                auto raw_points = pc->RawPointsProxy<Eigen::Vector3d>();
                auto world_points = pc->WorldPointsProxy<Eigen::Vector3d>();

                if (pc->HasTimestamps() && trajectory.Poses().size() >= 2)
                    pc->RawPointsToWorldPoints(trajectory);
                else
                    pc->RawPointsToWorldPoints(trajectory.Poses().front().pose);

                if (!pc->HasTimestamps()) {
                    pc->AddDefaultTimestampsField();
                    auto timestamps = pc->Timestamps<double>();
                    std::fill(timestamps.begin(), timestamps.end(), std::numeric_limits<double>::min());
                }
            }
            if (!pc->HasTimestamps())
                pc->AddDefaultTimestampsField();


            // Insert Points into the point cloud
            std::map<size_t, std::set<slam::Voxel>> voxels_to_update; //< Keep track of the voxels modified
            std::set<size_t> selected_indices; //< Keep track of the points inserted
            auto xyz = pc->WorldPointsProxy<Eigen::Vector3d>();
            auto timestamps = pc->TimestampsProxy<double>();


            for (auto pidx(0); pidx < xyz.size(); pidx++) {
                Eigen::Vector3d wpoint = xyz[pidx];
                double t = timestamps[pidx];
                for (auto map_idx(0); map_idx < options_.resolutions.size(); map_idx++) {
                    auto voxel = InsertPointInVoxelMap(wpoint, map_idx, fidx, pidx, t);
                    if (voxel) {
                        voxels_to_update[map_idx].insert(*voxel);
                        selected_indices.insert(pidx);
                    }
                }
            }


            // Compute the normals for Voxel Blocks to Update
            // TODO: Measure the time of each iterations ?
            for (auto &[map_id, voxels]: voxels_to_update) {
                auto &map = voxel_maps_[map_id].map;
                for (auto &voxel: voxels) {
                    auto &voxel_block = map[voxel];

                    if (voxel_block.points.size() >= 5) {
                        voxel_block.ComputeNeighborhood(slam::ALL_BUT_KDTREE);

                        for (auto &point: voxel_block.points) {
                            point.normal = voxel_block.description.normal;
                            point.is_normal_computed = true;
                            if (frame_id_to_frame.find(point.frame_id) != frame_id_to_frame.end()) {
                                // Orient the normal using the pose of the source frame
                                auto &src_frame = frame_id_to_frame[point.frame_id];
                                auto &begin = src_frame.poses.Poses().front();
                                if ((point.xyz - begin.TrRef()).dot(point.normal) > 0.) {
                                    point.normal = -point.normal;
                                }
                                point.is_normal_oriented = true;
                            } else
                                point.is_normal_oriented = false;
                        }
                    }
                }
            }


            frame.pointcloud = pc;
            if (frame.pointcloud->HasTimestamps()) {
                auto _timestamps = frame.pointcloud->TimestampsProxy<double>();
                auto [min_it, max_it] = std::minmax_element(_timestamps.begin(), _timestamps.end());
                frame.min_t = *min_it;
                frame.max_t = *max_it;
            }

            frame_indices_.push_back(frame_id_count_ - 1);
            // Remove old point clouds in memory
            while (frame_indices_.size() > options_.max_frames_to_keep) {
                auto oldest_idx = frame_indices_.front();
                frame_indices_.pop_front();
                frame_id_to_frame[oldest_idx].pointcloud = nullptr;
            }

        }

        // TODO:
        //  -- Remove Points
        //  -- Fast and Strong Queries

        // Returns the voxel where the point was inserted
        std::optional<slam::Voxel> InsertPointInVoxelMap(const Eigen::Vector3d &point, size_t map_index,
                                                         size_t frame_idx, size_t pidx,
                                                         double timestamp = std::numeric_limits<double>::min()) {
            const auto &[resolution, min_dist, max_num_points] = options_.resolutions[map_index];
            auto &hash_map_ = voxel_maps_[map_index];
            slam::Voxel voxel = slam::Voxel::Coordinates(point, resolution);

            if (hash_map_.map.find(voxel) == hash_map_.map.end()) {
                hash_map_.map[voxel].points.reserve(max_num_points);
                hash_map_.map[voxel].points.push_back(
                        PointType{point, Eigen::Vector3d::Zero(), timestamp, frame_idx, pidx});
                hash_map_.num_points++;
                return voxel;
            }
            auto &voxel_block = hash_map_.map[voxel];
            if (voxel_block.points.size() < max_num_points) {
                double sq_dist_min_to_points = std::numeric_limits<double>::max();
                // Insert a point only if it is greader than the min distance between points
                for (int i(0); i < voxel_block.points.size(); ++i) {
                    auto &_point = voxel_block.points[i];
                    double sq_dist = (_point.xyz.cast<double>() - point).squaredNorm();
                    if (sq_dist < sq_dist_min_to_points) {
                        sq_dist_min_to_points = sq_dist;
                    }
                }
                if (sq_dist_min_to_points > (min_dist * min_dist)) {
                    voxel_block.points.push_back({point, Eigen::Vector3d::Zero(), timestamp, frame_idx, pidx});
                    hash_map_.num_points++;
                    return voxel;
                }
            }
            return {};
        }

        // @brief   Clears the map
        void ClearMap() override { Reset(options_, false); };

        // @brief   Adds a Point Cloud to the map
        void InsertPointCloud(const slam::PointCloud &cloud,
                              std::vector<size_t> &out_selected_points) override {
            InsertPointCloud(cloud, {slam::Pose()}, out_selected_points);
        };

        // @brief   Removes elements of the map far from the given location
        void RemoveElementsFarFromLocation(const Eigen::Vector3d &location, double distance) override {
            // Iterate over all voxels and suppress the voxels to remove
            for (auto map_idx = 0; map_idx < voxel_maps_.size(); map_idx++) {
                std::set<slam::Voxel> voxels_to_remove;
                auto &map = voxel_maps_[map_idx].map;
                for (auto &[voxel, neighborhood]: voxel_maps_[map_idx].map) {
                    if (neighborhood.points.empty())
                        voxels_to_remove.insert(voxel);
                    if ((neighborhood.points.front().xyz - location).norm() > distance)
                        voxels_to_remove.insert(voxel);
                }

                for (auto &voxel: voxels_to_remove) {
                    voxel_maps_[map_idx].num_points -= map[voxel].points.size();
                    map.erase(voxel);
                }
            }
        };

        void Reset(const Options &options, bool keep_frames = false) {
            options_ = options;
            voxel_maps_.resize(0);
            voxel_maps_.resize(options.resolutions.size());

            if (keep_frames) {
                throw std::runtime_error("Not implemented");
                // TODO: Rebuild the map from the frames in memory
            } else {
                frame_id_count_ = 0;
                frame_id_to_frame.clear();
            }
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Export API
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /*!
         * @brief Returns the number of points in the voxel map of least resolution
         */
        size_t NumPoints() const override { return voxel_maps_.front().num_points; }

        /*!
         * @brief Returns the point cloud of the voxel map of least resolution
         */
        slam::PointCloudPtr MapAsPointCloud() const override { return GetMapPoints(0); }

        int NumVoxelMaps() const { return options_.resolutions.size(); }

        slam::PointCloudPtr GetMapPoints(size_t map_idx) const {
            auto &map = voxel_maps_[map_idx];

            auto pc = slam::PointCloud::DefaultXYZPtr<double>();
            pc->resize(map.num_points);
            pc->AddDefaultNormalsField();
            pc->AddDefaultTimestampsField();
            pc->SetWorldPointsField(slam::PointCloud::Field{pc->GetXYZField()});
            auto xyz = pc->XYZ<double>();
            auto normals = pc->NormalsProxy<Eigen::Vector3d>();
            size_t idx = 0;
            for (auto &[_, block]: map.map) {
                for (auto &point: block.points) {
                    CHECK(idx < map.num_points);
                    xyz[idx] = point.xyz;
                    normals[idx] = point.normal;

                    idx++;
                }
            }

            return pc;
        }

        /* @brief Returns all points visible from a sensor location */
        slam::PointCloudPtr GetVisibleMapPoints(size_t map_idx,
                                                const Eigen::Vector3d &view_point) const {
            auto &map = voxel_maps_[map_idx];
            auto pc = slam::PointCloud::DefaultXYZPtr<double>();
            pc->reserve(map.num_points);
            pc->AddDefaultNormalsField();
            pc->AddDefaultTimestampsField();
            pc->SetWorldPointsField(slam::PointCloud::Field{pc->GetXYZField()});
            auto xyz = pc->XYZ<double>();
            auto normals = pc->NormalsProxy<Eigen::Vector3d>();
            size_t idx = 0;
            for (auto &[_, block]: map.map) {
                for (auto &point: block.points) {
                    CHECK(idx < map.num_points);

                    bool add_point = true;
                    if (point.is_normal_oriented && point.is_normal_oriented) {
                        double scalar = point.normal.dot(point.xyz - view_point);
                        if (scalar < 0.) {
                            pc->resize(idx + 1);
                            xyz[idx] = point.xyz;
                            normals[idx] = point.normal;
                            idx++;
                        }
                    }
                }
            }
            return pc;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// QUERY API
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /*!
         * @brief Returns the search params adapted to the given radius
         */
        SearchParams SearchParamsFromRadiusSearch(double radius) const {
            SearchParams params;
            auto it = std::lower_bound(options_.resolutions.begin(),
                                       options_.resolutions.end(), radius,
                                       [](const ResolutionParam &lhs, double radius) {
                                           return lhs.resolution <= radius;
                                       });
            auto idx = std::max(int(0),
                                int(std::distance(options_.resolutions.begin(), it)) - 1);
            params.radius = radius;
            params.map_id = idx;
            double resolution = options_.resolutions[idx].resolution;
            params.voxel_resolution = resolution;
            params.voxel_neighborhood = std::ceil(radius / resolution);

            return params;
        }

        std::vector<slam::Neighborhood> ComputeNeighborhoods(const std::vector<Eigen::Vector3d> &queries,
                                                             const std::vector<double> radiuses,
                                                             int max_num_neighbors,
                                                             bool nearest_neighbors,
                                                             Eigen::Vector3d *sensor_location) const override {
            SLAM_CHECK_STREAM(radiuses.size() == queries.size(),
                              "Invalid Parameters, size of queries and radiuses do not match");
            std::vector<slam::Neighborhood> neighborhoods(queries.size());
            for (size_t i = 0; i < queries.size(); ++i) {
                RadiusSearchInPlace(queries[i], neighborhoods[i], radiuses[i], max_num_neighbors,
                                    nearest_neighbors, sensor_location);
            }
            return neighborhoods;
        }

        void RadiusSearchInPlace(const Eigen::Vector3d &query,
                                 slam::Neighborhood &neighborhood,
                                 double radius, int max_num_neighbors,
                                 bool nearest_neighbors,
                                 Eigen::Vector3d *sensor_location) const override {
            neighborhood.points.resize(0);
            neighborhood.points.reserve(max_num_neighbors);
            const SearchParams params = SearchParamsFromRadiusSearch(radius);

            const auto &hash_map_ = voxel_maps_[params.map_id].map;
            const double voxel_size = params.voxel_resolution;
            const int nb_voxels_visited = params.voxel_neighborhood;
            const double max_neighborhood_radius = params.radius;
            slam::Voxel voxel = slam::Voxel::Coordinates(query, voxel_size);
            int kx = voxel.x;
            int ky = voxel.y;
            int kz = voxel.z;

            PointType neighbor;
            priority_queue_t priority_queue;
            size_t num_points_skipped = 0;
            for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
                for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                        voxel.x = kxx;
                        voxel.y = kyy;
                        voxel.z = kzz;

                        auto search = hash_map_.find(voxel);
                        if (search != hash_map_.end()) {
                            const auto &voxel_block = search.value();
                            for (int i(0); i < voxel_block.points.size(); ++i) {
                                neighbor = voxel_block.points[i];
                                if (options_.select_valid_normals_direction && sensor_location &&
                                    neighbor.is_normal_oriented && neighbor.is_normal_computed) {
                                    // Remove points which have incompatible normals
                                    double scalar = (*sensor_location - query).dot(neighbor.normal);
                                    if (scalar < 0.) {
                                        num_points_skipped++;
                                        continue;
                                    }
                                }
                                double distance = (neighbor.xyz - query).norm();
                                if (distance > max_neighborhood_radius)
                                    continue;
                                if (priority_queue.size() == max_num_neighbors) {
                                    if (distance < std::get<0>(priority_queue.top())) {
                                        priority_queue.pop();
                                        priority_queue.emplace(distance, neighbor.xyz, voxel);
                                    }
                                } else
                                    priority_queue.emplace(distance, neighbor.xyz, voxel);

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

        slam::Neighborhood RadiusSearch(const Eigen::Vector3d &query, double radius,
                                        int max_num_neighbors, bool nearest_neighbors,
                                        Eigen::Vector3d *sensor_location) const override {
            slam::Neighborhood neighborhood;
            RadiusSearchInPlace(query, neighborhood, radius, max_num_neighbors, nearest_neighbors, sensor_location);
            return neighborhood;
        }

        /**
         * @brief  Constructs a neighborhood in place for a given spatial query
         */
        void ComputeNeighborhoodInPlace(const Eigen::Vector3d &query, int max_num_neighbors,
                                        slam::Neighborhood &neighborhood) const override {
            RadiusSearchInPlace(query, neighborhood, options_.default_radius, max_num_neighbors, true, nullptr);
        };

        // @brief   Returns a vector of neighborhood from a vector of queries
        std::vector<slam::Neighborhood> ComputeNeighborhoods(const std::vector<Eigen::Vector3d> &queries,
                                                             int max_num_neighbors) const override {

            std::vector<slam::Neighborhood> neighborhoods(queries.size());
            for (auto idx = 0; idx < neighborhoods.size(); ++idx) {
                ComputeNeighborhoodInPlace(queries[idx], max_num_neighbors, neighborhoods[idx]);
            }
            return neighborhoods;
        };

    private:
        size_t frame_id_count_ = 0;
        struct PointType {
            Eigen::Vector3d xyz;
            Eigen::Vector3d normal = Eigen::Vector3d::Zero();
            double timestamp = std::numeric_limits<double>::min();
            size_t frame_id = -1;
            size_t point_id = -1;

            bool is_normal_computed = false;
            bool is_normal_oriented = false;
        };

        struct _PointConversion {
        private:
            typedef slam::conversion_traits<PointType, Eigen::Vector3d, slam::reference_conversion_tag> __conversion_traits;
        public:
            typedef __conversion_traits::value_type value_type;
            typedef __conversion_traits::value_reference value_reference;
            typedef __conversion_traits::value_const_reference value_const_reference;
            typedef __conversion_traits::source_value_type source_value_type;
            typedef __conversion_traits::source_reference source_reference;
            typedef __conversion_traits::source_const_reference source_const_reference;
            typedef __conversion_traits::conversion_category conversion_category;

            value_const_reference &operator()(source_const_reference point) const {
                return point.xyz;
            }

            value_reference &operator()(source_reference &point) const {
                return point.xyz;
            }
        } conversion_;

        typedef slam::TNeighborhood<PointType, _PointConversion> _Neighborhood;

        struct Frame {
            slam::PointCloudPtr pointcloud = nullptr;
            slam::LinearContinuousTrajectory poses;
            double min_t = std::numeric_limits<double>::min(), max_t = std::numeric_limits<double>::min(); // Min and max timestamp
        };

        Options options_;

        typedef _Neighborhood VoxelBlock;
        struct VoxelHashMap {
            size_t num_points = 0;
            tsl::robin_map<slam::Voxel, VoxelBlock> map;
        };

        using pair_distance_t = std::tuple<double, Eigen::Vector3d, slam::Voxel>;

        struct __Comparator {
            bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
                return std::get<0>(left) < std::get<0>(right);
            }
        };

        typedef std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, __Comparator> priority_queue_t;

        std::list<size_t> frame_indices_;
        std::map<size_t, Frame> frame_id_to_frame;
        std::vector<VoxelHashMap> voxel_maps_;
    };


    /*!
     * @brief Reads a Map Options from a YAML::Node
     */
    std::shared_ptr<ct_icp::IMapOptions> yaml_to_map_options(const YAML::Node &node);

} // namespace ct_icp


#endif //CT_ICP_MAP_H
