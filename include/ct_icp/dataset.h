#ifndef CT_ICP_DATASET_HPP
#define CT_ICP_DATASET_HPP

#include <memory>
#include "types.h"
#include <filesystem>

#include "SlamUtils/io.h"
#include "SlamUtils/trajectory.h"

namespace fs = std::filesystem;

namespace ct_icp {

    enum DATASET {
        KITTI_raw = 0,
        KITTI_CARLA = 1,
        KITTI = 2,
        KITTI_360 = 3,
        NCLT = 4,
        HILTI = 5,
        PLY_DIRECTORY = 6
    };

    DATASET DATASETFromString(const std::string &);

    std::string DATASETEnumToString(DATASET dataset);

    bool IsDrivingDataset(DATASET dataset);

    /**
     * An ADatasetSequence allows access to the frames for a sequence of a dataset
     */
    class ADatasetSequence {
    public:
        virtual ~ADatasetSequence() = 0;

        struct Frame {

            std::vector<slam::WPoint3D> points;

            std::optional<slam::Pose> begin_pose{};

            std::optional<slam::Pose> end_pose{};

            [[nodiscard]] inline bool HasGroundTruth() const {
                return begin_pose.has_value() && end_pose.has_value();
            }

        };

        // Whether the sequence contains another frame
        [[nodiscard]] virtual bool HasNext() const { return false; };


        // Returns the number of frames (-1 if the total number of frames is unknown)
        [[nodiscard]] virtual size_t NumFrames() const = 0;

        // Returns the next frame (is the sequence contains one)
        // Applies the eventual filter defined on the frame
        virtual Frame NextFrame();

        // Returns a frame at `index`. Throws an exception if the dataset does not support Random Access
        // Applies the eventual filter defined on the frame
        virtual Frame GetFrame(size_t index) const;

        // Sets a filter on the frame
        void SetFilter(std::function<void(std::vector<slam::WPoint3D> &)> &&filter);

        // Remove the filter on the frame
        void ClearFilter();

        virtual void SetInitFrame(int frame_index) = 0;

        void SetMaxNumFrames(int max_num_frames) { max_num_frames_ = max_num_frames; };

        // Whether the dataset support random access
        virtual bool WithRandomAccess() const;

        // Returns the ground truth if the dataset has a ground truth
        virtual std::optional<std::vector<slam::Pose>> GroundTruth() { return {}; };

    protected:

        // Returns the next frame (is the sequence contains one)
        virtual Frame NextUnfilteredFrame() = 0;

        [[nodiscard]] virtual Frame GetUnfilteredFrame(size_t index) const;

        int max_num_frames_ = -1;
        int init_frame_id_ = 0; // The initial frame index of the sequence
        int current_frame_id_ = 0; // The current frame index of the iterator
        std::optional<std::function<void(std::vector<slam::WPoint3D> &)>> filter_{};
    };

    inline std::string DefaultFilePattern(size_t index_file, int zero_padding = 6) {
        std::stringstream ss;
        ss << "frame_" << std::setw(zero_padding) << std::setfill('0') << index_file << ".ply";
        return ss.str();
    }

    /**
     * A Generic Sequence to iterate over a directory consisting of point cloud PLY files
     */
    class PLYDirectory : public ADatasetSequence {
    public:
        typedef std::function<std::string(size_t)> PatternFunctionType;

        explicit PLYDirectory(fs::path &&root_path, size_t expected_size, PatternFunctionType &&optional_pattern);

        static PLYDirectory FromDirectoryPath(const std::string &dir_path);

        static std::shared_ptr<PLYDirectory> PtrFromDirectoryPath(const std::string &dir_path);

        [[nodiscard]] bool HasNext() const override;

        Frame NextUnfilteredFrame() override;

        void SetGroundTruth(std::vector<Pose> &&poses);

        void SetFilePattern(size_t expected_size, std::function<std::string(size_t)> &&file_pattern);

        // Returns the number of frames (-1 if the total number of frames is unknown)
        [[nodiscard]] size_t NumFrames() const;

        // Returns a frame at `index`. Throws an exception if the dataset does not support Random Access
        [[nodiscard]] Frame GetUnfilteredFrame(size_t index) const override;;

        // Whether the dataset support random access
        [[nodiscard]] bool WithRandomAccess() const override { return true; };

        // Returns the ground truth if the dataset has a ground truth
        std::optional<std::vector<slam::Pose>> GroundTruth() override;;

        REF_GETTER(Schema, schema_)

        explicit PLYDirectory(fs::path &&root_path,
                              std::vector<std::string> &&file_names);

        void SetInitFrame(int frame_index) override;

    private:
        size_t full_sequence_size_ = -1;
        std::vector<std::string> file_names_;
        fs::path root_dir_path_;
        slam::PointCloudSchema schema_;
        std::optional<slam::LinearContinuousTrajectory> ground_truth_{};
        std::optional<PatternFunctionType> file_pattern_{};
    };

    struct SequenceOptions {
        std::string sequence_name; // The name of the sequence
        int start_frame_id = 0; // The first frame of the sequence
        int max_num_frames = -1; // The maximum number of frames to run for (-1 to run on all frames)
    };

    struct DatasetOptions {

        DATASET dataset;

        std::string root_path;

        bool fail_if_incomplete = false; // Whether to fail if all sequences are not present on disk

        double min_dist_lidar_center = 3.0; // Threshold to filter points too close to the LiDAR center

        double max_dist_lidar_center = 100.0; // Threshold to filter points too far to the LiDAR center

        int nclt_num_aggregated_pc = 220; // The number of hits to aggregate for NCLT Dataset

        bool use_all_datasets = true; // Whether to use all sequences, or only the ones specified in param `sequence_options`

        std::vector<SequenceOptions> sequence_options;

    };

    struct SequenceInfo {

        std::string sequence_name;

        int sequence_id = -1;

        int sequence_size = -1;

        bool with_ground_truth = false;

    };

    /**
     * An ADataset regroups multiple sequences
     */
    class Dataset {
    public:
        // Returns all detected sequences
        std::vector<SequenceInfo> AllSequenceInfo() const;

        // Returns all detected sequences with ground truth
        std::vector<SequenceInfo> AllSequenceInfoWithGT() const;

        // Returns all sequences
        std::vector<std::shared_ptr<ADatasetSequence>> AllSequences() const;

        // Returns all sequences with ground truth
        std::vector<std::shared_ptr<ADatasetSequence>> AllSequencesWithGroundTruth() const;

        inline DATASET DatasetType() const { return dataset_; };

        // Returns whether the dataset has a sequence named `sequence_name`
        bool HasSequence(const std::string &sequence_name) const;

        // Returns whether the dataset has a ground truth for the sequence `sequence_name`
        bool HasGroundTruth(const std::string &sequence_name) const;

        // Returns the ground truth poses given a sequence name
        std::vector<Pose> GetGroundTruth(const std::string &sequence_name) const;

        static Dataset LoadDataset(const DatasetOptions &options);

    private:
        Dataset(std::vector<std::shared_ptr<ADatasetSequence>> &&dataset_sequences,
                std::vector<SequenceInfo> &&sequence_infos) :
                sequence_infos_(std::move(sequence_infos)),
                dataset_sequences_(std::move(dataset_sequences)) {
            int i(0);
            for (auto &seq_info: sequence_infos_) {
                map_seq_info_seq_id_[seq_info.sequence_name] = i;
                seq_info.sequence_id = i++;
            }
        }


        DATASET dataset_;
        std::vector<std::shared_ptr<ADatasetSequence>> dataset_sequences_;
        std::vector<SequenceInfo> sequence_infos_;
        std::map<std::string, size_t> map_seq_info_seq_id_;
    };

    // Reads the poses from NCLT ground truth poses
    std::vector<slam::Pose> ReadNCLTPoses(const std::string &file_path);
}


#endif //CT_ICP_DATASET_HPP
