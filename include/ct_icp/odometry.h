#ifndef CT_ICP_ODOMETRY_H
#define CT_ICP_ODOMETRY_H

#include "ct_icp/ct_icp.h"
#include "ct_icp/algorithm/sampling.h"
#include "ct_icp/map.h"

#include <map>

namespace ct_icp {

    enum MOTION_COMPENSATION {
        NONE = 0,              // No compensation of the motion
        CONSTANT_VELOCITY = 1, // Initial distortion of the point cloud based on the estimated velocity
        ITERATIVE = 2,         // Iterative refinement after each ICP iteration
        CONTINUOUS = 3         // Continuous estimation of the pose
    };

    enum INITIALIZATION {
        INIT_NONE = 0,
        INIT_CONSTANT_VELOCITY = 1
    };

    namespace sampling {
        enum SAMPLING_OPTION {
            NONE,
            GRID,
            ADAPTIVE
        };
    }

    struct OdometryOptions {

        /* ---------------------------------------------------------------------------------------------------------- */
        // Main Options

        CTICPOptions ct_icp_options;

        MOTION_COMPENSATION motion_compensation = CONTINUOUS;

        INITIALIZATION initialization = INIT_CONSTANT_VELOCITY;

        /* ---------------------------------------------------------------------------------------------------------- */
        // Initialization Regimen

        double init_voxel_size = 0.2;
        double init_sample_voxel_size = 1.0;
        int init_num_frames = 20; // The number of frames defining the initialization of the map

        /* ---------------------------------------------------------------------------------------------------------- */
        // SAMPLING Options
        double sample_voxel_size = 1.5;
        int max_num_keypoints = -1;

        sampling::SAMPLING_OPTION sampling = sampling::GRID;

        ct_icp::AdaptiveGridSamplingOptions adaptive_options;

        /* ---------------------------------------------------------------------------------------------------------- */
        // MAP OPTIONS
        std::shared_ptr<ct_icp::IMapOptions> map_options = nullptr;

        std::shared_ptr<ct_icp::INeighborStrategyOptions> neighborhood_strategy = nullptr;

        /// OLD PARAMETERS
        // Topology Options
        double size_voxel_map = 1.0;

        int max_num_points_in_voxel = 20; // The maximum number of points in a voxel

        // Search Options
        short voxel_neighborhood = 1;

        double max_radius_neighborhood = 0.8;

        double min_distance_points = 0.1; // The minimal distance between points in the map

        /* ---------------------------------------------------------------------------------------------------------- */
        // FRAME CONSRUCTION OPTIONS

        double voxel_size = 0.5;

        double max_distance = 100.0; // The threshold on the voxel size to remove points from the map

        // TODO: Validity check options
        double distance_error_threshold = 5.0; // The Ego-Motion Distance considered as an error
        double orientation_error_threshold = 30.; // The Ego Orientation considered as an error
        bool quit_on_error = true;

        /* ---------------------------------------------------------------------------------------------------------- */
        /* ROBUST REGIMEN OPTION                                                                                      */

        // Whether to assess the quality of the registration,
        // And try a new registration with more conservative parameters in case of failure
        int robust_minimal_level = 0;
        bool robust_registration = false;
        double robust_full_voxel_threshold = 0.7;
        double robust_empty_voxel_threshold = 0.1;
        double robust_neighborhood_min_dist = 0.10; // The minimum relative distance to launch a robust neighborhood test
        double robust_neighborhood_min_orientation = 0.1; // The minimum relative orientation to launch a robust neighborhood
        // Threshold on the relative transform (all motion at 10Hz should be below this value)
        double robust_relative_trans_threshold = 1.0;
        bool robust_fail_early = false; // Stop iterations if the final assessment of the registration is unsucessful
        int robust_num_attempts = 6;
        int robust_num_attempts_when_rotation = 2;
        short robust_max_voxel_neighborhood = 3;
        double robust_threshold_ego_orientation = 3; // Angle in degrees
        double robust_threshold_relative_orientation = 3; // Angle in degrees

        double insertion_ego_rotation_threshold = 3; // Ego rotation in degrees of frame to avoid inserting
        double insertion_threshold_frames_skipped = 5; // Threshold on number of frames skipped (inserts a frame after it was skipped)
        double insertion_cum_distance_threshold = 0.8; // Threshold on cumulative distance threshold
        double insertion_cum_orientation_threshold = 5; // Threshold on cumulative orientation

        /* ---------------------------------------------------------------------------------------------------------- */
        /*  DEBUG AND OUTPUT PARAMS                                                                                   */

        bool always_insert = false; // Always insert into the map by the Odometry Node (overseeds do_not_insert)
        bool do_no_insert = false; // No insertion in the map by the Odometry Node

        // Debug Parameters
        bool debug_print = true; // Whether to print debug information into the console

        bool debug_viz = false; // Whether to display the Local Map in a window

        bool log_to_file = false;

        std::string log_file_destination = "/tmp/ct_icp.log";

        /* ---------------------------------------------------------------------------------------------------------- */
        /*  MOTION MODEL                                                                                              */
        PreviousFrameMotionModel::Options default_motion_model;
        bool with_default_motion_model = true;


        ////////////////////////
        /// DEFAULT PROFILES ///
        ////////////////////////

        // Returns the default parameters for driving scenarios
        // e.g. Used for the dataset KITTI
        static OdometryOptions DefaultDrivingProfile();

        // Returns the default parameters for sensor with high frequency acceleration changes
        // e.g. Used for the dataset NCLT
        static OdometryOptions DefaultRobustOutdoorLowInertia();

        // Returns the default parameters for a robust Driving Profile
        static OdometryOptions RobustDrivingProfile();
        // TODO: INDOOR

        OdometryOptions() {
            map_options = std::make_shared<ct_icp::MultipleResolutionVoxelMap::Options>();
            neighborhood_strategy = std::make_shared<ct_icp::DefaultNearestNeighborStrategy::Options>();
        }

    };

    class Odometry {
    public:

        // The Output of a registration, including metrics,
        struct RegistrationSummary {

            TrajectoryFrame frame, initial_frame;

            int sample_size = 0; // The number of points sampled

            int number_of_residuals = 0; // The number of keypoints used for ICP registration

            int robust_level = 0;

            double distance_correction = 0.0; // The correction between the last frame's end, and the new frame's beginning

            double relative_distance = 0.0; // The distance between the beginning of the new frame and the end

            double relative_orientation = 0.0; // The distance between the beginning of the new frame and the end

            double ego_orientation = 0.0; // The angular distance between the beginning and the end of the frame

            bool success = true; // Whether the registration was a success

            bool points_added = false; // Whether points were added to the map

            int number_of_attempts = 0; // The number of attempts at registering the new frame

            std::string error_message;

            std::vector<slam::WPoint3D> corrected_points; // Sampled points expressed in the initial frame

            std::vector<slam::WPoint3D> all_corrected_points; // Initial points expressed in the initial frame

            std::vector<slam::WPoint3D> keypoints; // Last Keypoints selected

            ICPSummary icp_summary; // The summary of the last ICP

            std::map<std::string, double> logged_values; //< Logging values for the summary

        };

        struct FrameInfo {
            int registered_fid = -1; // The index of the new frame (since the initial insertion of the frame)
            slam::frame_id_t frame_id = -1; // The frame index
            double begin_timestamp = -1., end_timestamp = -1.;
        };

        // @brief   An abstract Callback which can be registered to the Odometry and will be called at a specified
        //          Stages of the pipeline
        struct OdometryCallback {

            enum EVENT {
                BEFORE_ITERATION, //< Runs the callback before the iteration
                ITERATION_COMPLETED, //< Run the callback once an iteration has been completed
                FINISHED_REGISTRATION //< Runs the callback after the end of the iterations
            };

            // @brief   Execution method of the Callback with the instance who called the callback as argument
            virtual bool Run(
                    const Odometry &odometry,
                    const std::vector<slam::WPoint3D> &current_frame,
                    const std::vector<slam::WPoint3D> *keypoints = nullptr,
                    const RegistrationSummary *summary = nullptr) = 0;

        };

        explicit Odometry(const OdometryOptions &options);

        explicit Odometry(const OdometryOptions *options) : Odometry(*options) {}

        // Registers a new Frame to the Map (with custom motion model)
        RegistrationSummary RegisterFrame(const slam::PointCloud &frame,
                                          slam::frame_id_t frame_id,
                                          AMotionModel *motion_model = nullptr);

        // Registers a new Frame to the Map with an initial estimate
        RegistrationSummary RegisterFrameWithEstimate(const slam::PointCloud &frame,
                                                      const TrajectoryFrame &initial_estimate,
                                                      slam::frame_id_t frame_id,
                                                      AMotionModel *motion_model = nullptr);

        // Registers a new Frame to the Map
        RegistrationSummary RegisterFrame(const std::vector<slam::WPoint3D> &frame,
                                          AMotionModel *motion_model = nullptr);

        // Registers a new Frame to the Map with an initial estimate
        RegistrationSummary RegisterFrameWithEstimate(const std::vector<slam::WPoint3D> &frame,
                                                      const TrajectoryFrame &initial_estimate,
                                                      AMotionModel *motion_model = nullptr);

        // Returns the currently registered trajectory
        [[nodiscard]] std::vector<TrajectoryFrame> Trajectory() const;

        // Returns the Aggregated PointCloud of the Local Map
        [[nodiscard]] slam::PointCloudPtr GetMapPointCloud() const;

        // Num Points in the Map
        // Note: This requires a traversal of the whole map which is in O(n)
        [[nodiscard]] size_t MapSize() const;

        // Registers a Callback to the Odometry
        void RegisterCallback(OdometryCallback::EVENT event, OdometryCallback &callback);

        REF_GETTER(Map, *map_)

        // Resets the state of the odometry
        void Reset();

        // Resets the state of the odometry (changing the options)
        void Reset(const ct_icp::OdometryOptions &options);

        // Returns the pointer to the map
        std::shared_ptr<ct_icp::ISlamMap> GetMapPointer();

    private:
        std::map<OdometryCallback::EVENT, std::vector<OdometryCallback *>> callbacks_;
        std::vector<TrajectoryFrame> trajectory_;
        std::shared_ptr<ct_icp::ISlamMap> map_ = nullptr;
        std::shared_ptr<ct_icp::ANeighborhoodStrategy> neighborhood_strategy_ = nullptr;
        PreviousFrameMotionModel default_motion_model;
        int registered_frames_ = 0;
        int robust_num_consecutive_failures_ = 0;
        bool suspect_registration_error_ = false;
        int next_robust_level_ = 0;
        OdometryOptions options_;
        std::ostream *log_out_ = nullptr;
        std::unique_ptr<std::ofstream> log_file_ = nullptr;
        std::mt19937_64 g_;

        // A Helper class which pilots the robustness of the
        // By evaluating the quality of the registration
        struct RobustRegistrationAttempt {

            // Sets an initial Robust Level
            void SetRobustLevel(int level);

            TrajectoryFrame &CurrentFrame() { return summary.frame; }

            // Heuristic to increase the robustness level by doing more work
            // Todo : Seriously need to improve the paradigm of robustness !
            void IncreaseRobustnessLevel();

            RobustRegistrationAttempt(
                    int index_frame,
                    const OdometryOptions &options,
                    const TrajectoryFrame &initial_estimate);

            int robust_level = 0;
            double sample_voxel_size;
            slam::frame_id_t index_frame;
            TrajectoryFrame previous_frame;

            const TrajectoryFrame &initial_estimate_;
            const ct_icp::OdometryOptions &options_;
            ct_icp::CTICPOptions registration_options;
//            VoxelHashMap::SearchOptions search_options;
            RegistrationSummary summary;
        };

        struct FrameInsertionTracker {
            size_t last_inserted_frame_idx = 0;
            double cum_distance_since_insertion = 0.;
            double cum_orientation_change_since_insertion = 0.;

            int skipped_frames = 0;
            int total_insertions = 0;

            void InsertFrame(size_t frame_id) {
                last_inserted_frame_idx = frame_id;
                cum_orientation_change_since_insertion = 0.;
                cum_distance_since_insertion = 0.;
                skipped_frames = 0;
                total_insertions++;
            }

            void SkipFrame() { skipped_frames++; }

            friend std::ostream &operator<<(std::ostream &os, const FrameInsertionTracker &tracker) {
                os << " Skipped Frames [" << tracker.skipped_frames << " / "
                   << tracker.options_.insertion_threshold_frames_skipped << "]"
                   << "  Total Insertions: " << tracker.total_insertions;
                return os;
            };

            const OdometryOptions &options_;

            explicit FrameInsertionTracker(const OdometryOptions &options) : options_(options) {}

        } insertion_tracker_;

        void ComputeSummaryMetrics(RegistrationSummary &summary, size_t index_frame);

        void RobustRegistration(std::vector<slam::WPoint3D> &frame,
                                FrameInfo frame_info,
                                RegistrationSummary &registration_summary,
                                AMotionModel *motion_model = nullptr);

        void LogInitialization(std::vector<slam::WPoint3D> &sampled_frame,
                               FrameInfo &frame_info, std::ostream *out) const;

        void LogSummary(RegistrationSummary &summary) const;;

        // Iterate over the callbacks registered
        void IterateOverCallbacks(OdometryCallback::EVENT event,
                                  const std::vector<slam::WPoint3D> &current_frame,
                                  const std::vector<slam::WPoint3D> *keypoints = nullptr,
                                  const RegistrationSummary *summary = nullptr);

        // Initialize the Frame.
        // Returns the set of selected keypoints sampled via grid sampling
        std::vector<slam::WPoint3D> InitializeFrame(const slam::PointCloud &const_frame,
                                                    FrameInfo frame_info);

        // Registers a frame after the motion was initialized
        // When the Robust Registration profile is activated, it can call TryRegister
        // Multiple times changing the options in order to increase the chance of registration
        RegistrationSummary DoRegister(const slam::PointCloud &frame,
                                       FrameInfo frame_info,
                                       AMotionModel *motion_model = nullptr);

        // Tries to register a frame given a set of options
        void TryRegister(std::vector<slam::WPoint3D> &frame,
                         FrameInfo frame_info,
                         CTICPOptions &options,
                         RegistrationSummary &registration_summary,
                         double sample_voxel_size,
                         AMotionModel *motion_model = nullptr);

        // Insert a New Trajectory Frame, and initializes the motion for this new frame
        void InitializeMotion(FrameInfo frame_info, const TrajectoryFrame *initial_estimate = nullptr);

        // Try to insert Points to the map
        // Returns false if it fails
        bool AssessRegistration(const std::vector<slam::WPoint3D> &points, RegistrationSummary &summary,
                                std::ostream *log_stream = nullptr) const;

        // Inspect the Summary to determine whether point should be added to the map
        void UpdateMap(RegistrationSummary &summary, int registered_fid);

        friend class OdometryCallback;
        friend class OdometryReactor;
        friend class InertialCTSlamReactor;
    };

} // namespace ct_icp


#endif //CT_ICP_ODOMETRY_H
