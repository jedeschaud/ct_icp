#ifndef CT_ICP_DATASET_HPP
#define CT_ICP_DATASET_HPP

#include <memory>
#include <fstream>

#include "types.h"
#include "ct_icp/utils.h"

#include <SlamCore/io.h>
#include <SlamCore/trajectory.h>
#include <SlamCore/experimental/synthetic.h>
#include <SlamCore/pointcloud.h>

namespace ct_icp
{

    enum DATASET
    {
        KITTI_raw = 0,
        KITTI_CARLA = 1,
        KITTI = 2,
        KITTI_360 = 3,
        NCLT = 4,
        HILTI_2021 = 5,
        HILTI_2022 = 6,
        PLY_DIRECTORY = 7,
        SYNTHETIC = 8,
        CUSTOM = 9,
        INVALID = -1
    };

    DATASET DATASETFromString(const std::string &);

    std::string DATASETEnumToString(DATASET dataset);

    bool IsDrivingDataset(DATASET dataset);

    /*! @brief A SequenceInformation */
    struct SequenceInfo
    {
        std::string sequence_name = "Unknown"; //< The name of the sequence
        std::string label = "Unknown";         //< Label of the sequence in the dataset
        int sequence_id = -1;                  //< The id of the sequence in the dataset
        int sequence_size = -1;                //< The size of the sequence
        bool with_ground_truth = false;        //< Whether the sequence has ground truth
    };

    /**
     * @brief An ADatasetSequence allows access to the frames for a sequence of a dataset
     */
    class ADatasetSequence
    {
    public:
        virtual ~ADatasetSequence() = 0;

        typedef LidarIMUFrame Frame;

        // Whether the sequence contains another frame
        [[nodiscard]] virtual bool HasNext() const { return false; };

        // Returns the number of frames (-1 if the total number of frames is unknown)
        [[nodiscard]] virtual size_t NumFrames() const = 0;

        // Returns the next frame (is the sequence contains one)
        // Applies the eventual filter defined on the frame
        virtual Frame NextFrame();

        // Skips the next frame
        virtual void SkipFrame();

        // Returns a frame at `index`. Throws an exception if the dataset does not support Random Access
        // Applies the eventual filter defined on the frame
        virtual Frame GetFrame(size_t index) const;

        // Sets a filter on the frame
        void SetFilter(std::function<void(slam::PointCloud &)> &&filter);

        // Remove the filter on the frame
        void ClearFilter();

        virtual void SetInitFrame(int frame_index) = 0;

        void SetMaxNumFrames(int max_num_frames) { max_num_frames_ = max_num_frames; };

        // Whether the dataset support random access
        virtual bool WithRandomAccess() const;

        // Returns the ground truth if the dataset has a ground truth
        virtual std::optional<std::vector<slam::Pose>> GroundTruth() { return {}; };

        // Returns whether the dataset has a ground truth
        virtual bool HasGroundTruth() const { return seq_info_.with_ground_truth; }

        const SequenceInfo &GetSequenceInfo() const
        {
            return seq_info_;
        }

        SequenceInfo &GetSequenceInfo()
        {
            return seq_info_;
        }

        explicit ADatasetSequence() {}

        // Returns the next frame (is the sequence contains one)
        virtual Frame NextUnfilteredFrame() = 0;

        // Register fields and apply the optional filters to the frame
        void ProcessFrame(Frame &frame) const;

        // Throws an exception by default (only for random access datasets e.g. sequence of files)
        [[nodiscard]] virtual Frame GetUnfilteredFrame(size_t index) const;

    protected:
        SequenceInfo seq_info_;
        int max_num_frames_ = -1;
        int init_frame_id_ = 0;    // The initial frame index of the sequence
        int current_frame_id_ = 0; // The current frame index of the iterator
        std::optional<std::function<void(slam::PointCloud &)>> filter_{};
    };

    inline std::string DefaultFilePattern(size_t index_file, int zero_padding = 6)
    {
        std::stringstream ss;
        ss << "frame_" << std::setw(zero_padding) << std::setfill('0') << index_file << ".ply";
        return ss.str();
    }

    /**
     * @brief A Synthetic Acquisition simulates the acquisition of a Depth Sensor in a synthetic environment
     */
    class SyntheticSequence : public ADatasetSequence
    {
    public:
        SyntheticSequence(slam::SyntheticSensorAcquisition &&sensor_acquisition,
                          std::vector<slam::Pose> &&gt_poses);

        static std::shared_ptr<SyntheticSequence> PtrFromDirectoryPath(const std::string &yaml_path);

        static std::shared_ptr<SyntheticSequence> PtrFromNode(const YAML::Node &node);

        [[nodiscard]] bool HasNext() const override;

        Frame NextUnfilteredFrame() override;

        // Returns the number of frames (-1 if the total number of frames is unknown)
        [[nodiscard]] size_t NumFrames() const override;

        // Returns a frame at `index`. Throws an exception if the dataset does not support Random Access
        [[nodiscard]] Frame GetUnfilteredFrame(size_t index) const override;
        ;

        // Whether the dataset support random access
        [[nodiscard]] bool WithRandomAccess() const override { return true; };

        // Returns the ground truth if the dataset has a ground truth
        std::optional<std::vector<slam::Pose>> GroundTruth() override { return ground_truth_poses_; };

        void SetInitFrame(int frame_index) override;

    private:
        struct Options
        {
            size_t num_points_per_primitives = 300; // The number of points per primitive to sample
            double max_lidar_distance = 100.0;      // The maximum distance to the lidar sensor for points of a new frame
            double sample_frequency = 10.;          // The frequency in Hz of the frame sampling
        } options_;
        slam::SyntheticSensorAcquisition acquisition_;
        std::vector<slam::Pose> ground_truth_poses_;
    };

    /**
     * @brief A abstract Sequence of files to iterate over
     */
    class AFileSequence : public ADatasetSequence
    {
    public:
        typedef std::function<std::string(size_t)> FilePatternFunctionType;
        typedef std::function<bool(const std::string &, const std::string &)> SortingFunctionType;

        /*! @brief Reads a Frame from the disk */
        virtual Frame ReadFrame(const std::string &filename) const = 0;

        /*! @brief  Defines the Sorting function  */
        void SetSortingFunction(SortingFunctionType &&function)
        {
            sorting_function = std::move(function);
            if (file_names_)
            {
                std::sort(file_names_->begin(), file_names_->end(), sorting_function);
            }
        }

        void SetGroundTruth(std::vector<Pose> &&poses);

        [[nodiscard]] bool HasNext() const override;

        Frame NextUnfilteredFrame() override;

        // Returns the number of frames (-1 if the total number of frames is unknown)
        [[nodiscard]] size_t NumFrames() const override;

        // Returns a frame at `index`. Throws an exception if the dataset does not support Random Access
        [[nodiscard]] Frame GetUnfilteredFrame(size_t index) const override;

        // Whether the dataset support random access
        [[nodiscard]] bool WithRandomAccess() const override { return true; };

        // Returns the ground truth if the dataset has a ground truth
        std::optional<std::vector<slam::Pose>> GroundTruth() override;

        void SetInitFrame(int frame_index) override;

        /*! @brief Returns the file paths for each file in the directory*/
        std::vector<std::string> GetFilePaths() const;

    protected:
        explicit AFileSequence(std::string &&dir_path,
                               const std::vector<std::string> &filenames_) : ADatasetSequence(),
                                                                             root_dir_path_(std::move(dir_path))
        {
            SLAM_CHECK_STREAM(fs::exists(root_dir_path_), "The Directory " << root_dir_path_ << " does not exist");
            file_names_ = filenames_;
            full_sequence_size_ = file_names_->size();
            std::sort(file_names_->begin(), file_names_->end(), sorting_function);
        }

        explicit AFileSequence(std::string &&root_path,
                               size_t expected_size, FilePatternFunctionType &&optional_pattern)
            : ADatasetSequence(),
              root_dir_path_(std::move(root_path))
        {
            file_pattern_ = {
                std::move(optional_pattern),
            };
            full_sequence_size_ = expected_size;
            seq_info_.sequence_size = int(expected_size);
        }

        SortingFunctionType sorting_function = [](const std::string &lhs, const std::string &rhs)
        { return lhs < rhs; };
        fs::path root_dir_path_;
        std::optional<std::vector<std::string>> file_names_;
        std::optional<slam::LinearContinuousTrajectory> ground_truth_{};
        std::optional<FilePatternFunctionType> file_pattern_;
        size_t full_sequence_size_ = -1;
    };

    /**
     * @brief A Generic Sequence to iterate over a directory consisting of point cloud PLY files
     */
    class PLYDirectory : public AFileSequence
    {
    public:
        explicit PLYDirectory(std::string &&root_path,
                              size_t expected_size, FilePatternFunctionType &&optional_pattern);

        explicit PLYDirectory(std::string &&root_path, std::vector<std::string> &&file_names);

        static PLYDirectory FromDirectoryPath(const std::string &dir_path,
                                              std::optional<SequenceInfo> seq_info = {});

        static std::shared_ptr<PLYDirectory> PtrFromDirectoryPath(const std::string &dir_path,
                                                                  std::optional<SequenceInfo> seq_info = {});

        // Read a PLY Frame from disk
        Frame ReadFrame(const std::string &filename) const override;

        REF_GETTER(GetSchemaMapper, mapper_)

    private:
        std::optional<slam::PLYSchemaMapper> mapper_ = {}; //< The default Schema Mapper.
    };

    struct SequenceOptions
    {
        std::string sequence_name; // The name of the sequence
        int start_frame_id = 0;    // The first frame of the sequence
        int max_num_frames = -1;   // The maximum number of frames to run for (-1 to run on all frames)
    };

    class NCLTIterator final : public ADatasetSequence
    {
    public:
        struct LidarPoint
        {
            Eigen::Vector3d xyz;
            double timestamp = -1.;

            static slam::ItemSchema DefaultSchema();

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        explicit NCLTIterator(const std::string &hits_filepath,
                              const std::string &sequence_name,
                              int num_aggregated_packets = 220);

        static std::shared_ptr<NCLTIterator> NCLTIteratorFromHitsFile(const std::string &vel_hits_file,
                                                                      const std::string &sequence_name);

        void SkipFrame() override;

        [[nodiscard]] bool HasNext() const override;

        void SetInitFrame(int frame_index) override;

        void SetGroundTruth(std::vector<Pose> &&poses);

        std::optional<std::vector<slam::Pose>> GroundTruth() override;

        size_t NumFrames() const override;

        Frame DoNext(bool jump_frame = false);

        Frame NextUnfilteredFrame() override;

        std::vector<LidarPoint> NextBatch(bool jump_batch);

        ~NCLTIterator() final;

    private:
        void OpenFile(const std::string &hits_file_path);

        void Close();

        int num_aggregated_pc_;

        std::optional<slam::LinearContinuousTrajectory> ground_truth_{};
        std::unique_ptr<std::ifstream> file = nullptr;
        std::string sequence_name_, hits_file_path_;
    };

    struct DatasetOptions
    {

        DATASET dataset;

        std::string root_path;

        bool fail_if_incomplete = false; // Whether to fail if all sequences are not present on disk

        double min_dist_lidar_center = 3.0; // Threshold to filter points too close to the LiDAR center

        double max_dist_lidar_center = 100.0; // Threshold to filter points too far to the LiDAR center

        int nclt_num_aggregated_pc = 220; // The number of hits to aggregate for NCLT Dataset

        bool use_all_datasets = true; // Whether to use all sequences, or only the ones specified in param `sequence_options`

        std::vector<SequenceOptions> sequence_options;
    };

    /**
     * An ADataset regroups multiple sequences
     */
    class Dataset
    {
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

        SequenceInfo GetSequenceInfo(const std::string &name) const;

        std::shared_ptr<ADatasetSequence> GetSequence(const std::string &name) const;

        // Returns whether the dataset has a sequence named `sequence_name`
        bool HasSequence(const std::string &sequence_name) const;

        // Returns whether the dataset has a ground truth for the sequence `sequence_name`
        bool HasGroundTruth(const std::string &sequence_name) const;

        // Returns the ground truth poses given a sequence name
        std::vector<Pose> GetGroundTruth(const std::string &sequence_name) const;

        static Dataset LoadDataset(const DatasetOptions &options);

        // Builds a dataset from a set of sequences
        static Dataset BuildCustomDataset(std::vector<std::shared_ptr<ADatasetSequence>> &&custom_sequences)
        {
            std::vector<SequenceInfo> sequence_infos;
            std::set<std::string> sequence_names;
            for (auto &sequence : custom_sequences)
            {
                auto &seq_info = sequence->GetSequenceInfo();
                if (sequence_names.find(seq_info.sequence_name) != sequence_names.end())
                {
                    SLAM_LOG(WARNING) << "A sequence with the name " << seq_info.sequence_name << " already exists" << std::endl;
                }
                sequence_infos.push_back(seq_info);
            }
            return Dataset(CUSTOM, std::move(custom_sequences), std::move(sequence_infos));
        };

    private:
        Dataset(DATASET dataset,
                std::vector<std::shared_ptr<ADatasetSequence>> &&dataset_sequences,
                std::vector<SequenceInfo> &&sequence_infos) : dataset_(dataset),
                                                              sequence_infos_(std::move(sequence_infos)),
                                                              dataset_sequences_(std::move(dataset_sequences))
        {
            int i(0);
            for (auto &seq_info : sequence_infos_)
            {
                map_seqname_to_id_[seq_info.sequence_name] = i;
                seq_info.sequence_id = i++;
            }
        }

        DATASET dataset_;
        std::vector<std::shared_ptr<ADatasetSequence>> dataset_sequences_;
        std::vector<SequenceInfo> sequence_infos_;
        std::map<std::string, size_t> map_seqname_to_id_;
    };

    // Reads the poses from NCLT ground truth poses
    std::vector<slam::Pose> ReadNCLTPoses(const std::string &file_path);

    // Reads the GT poses from HILTI dataset (in the imu frame)
    std::vector<slam::Pose> ReadHILTIPosesInIMUFrame(const std::string &file_path);

    // Reads the GT poses from the HILTI dataset (in the lidar frame)
    std::vector<slam::Pose> ReadHILTIPosesInLidarFrame(const std::string &file_path, DATASET hilti_dataset);
}

#endif // CT_ICP_DATASET_HPP
