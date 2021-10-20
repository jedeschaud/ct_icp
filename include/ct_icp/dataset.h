#ifndef CT_ICP_DATASET_HPP
#define CT_ICP_DATASET_HPP

#include <memory>
#include "types.h"


namespace ct_icp {

    enum DATASET {
        KITTI_raw = 0,
        KITTI_CARLA = 1,
        KITTI = 2,
        KITTI_360 = 3,
        NCLT = 4,
        PLY_DIRECTORY = 5
    };

    DATASET DATASETFromString(const std::string&);

    std::string DATASETEnumToString(DATASET dataset);

    class DatasetSequence {
    public:
        virtual ~DatasetSequence() = 0;

        virtual bool HasNext() const = 0;

        virtual std::vector<Point3D> Next() = 0;

        virtual size_t NumFrames() const {
            return -1;
        }

        virtual std::vector<Point3D> Frame(size_t index) const {
            throw std::runtime_error("Random Access is not supported");
        }

        virtual void SetInitFrame(int frame_index) {
            init_frame_id_ = frame_index;
        };

        virtual bool WithRandomAccess() const {
            return false;
        }

    protected:
        int init_frame_id_ = 0; // The initial frame index of the sequence
    };

    struct DatasetOptions {

        DATASET dataset;

        std::string root_path;

        bool fail_if_incomplete = false; // Whether to fail if all sequences are not present on disk

        double min_dist_lidar_center = 3.0; // Threshold to filter points too close to the LiDAR center

        double max_dist_lidar_center = 100.0; // Threshold to filter points too far to the LiDAR center

        int nclt_num_aggregated_pc = 220; // The number of hits to aggregate for NCLT Dataset

    };

    struct SequenceInfo {

        std::string sequence_name;

        int sequence_id = -1;

        int sequence_size = -1;

    };

    // Returns the Pairs sequence_id, sequence_size found on disk for the provided options
    std::vector<SequenceInfo> get_sequences(const DatasetOptions &);

    // Reads a PointCloud from the Dataset PLY_DIRECTORY
    std::vector<Point3D> read_ply_pointcloud(const DatasetOptions&, const std::string& path);

    // Reads a PointCloud from the Dataset KITTI_raw
    std::vector<Point3D> read_kitti_raw_pointcloud(const DatasetOptions &, const std::string &path);

    // Reads a PointCloud from the Dataset KITTI_CARLA
    std::vector<Point3D> read_kitti_carla_pointcloud(const DatasetOptions &, const std::string &path);

    // Reads a PointCloud from the Dataset KITTI
    std::vector<Point3D> read_kitti_pointcloud(const DatasetOptions &, const std::string &path);

    // Reads a PointCloud from the disk
    std::vector<Point3D> read_pointcloud(const DatasetOptions &, int sequence_id, int frame_id);

    // Converts A Trajectory Frame to the format of the ground truth
    // Note: This format depends on the dataset, and its evaluation protocol
    ArrayPoses transform_trajectory_frame(const DatasetOptions &, const std::vector<TrajectoryFrame> &,
                                          int sequence_id);

    // Returns the Sequence Name as a string given its id
    std::string sequence_name(const DatasetOptions &, int sequence_id);

    // Returns whether the Sequence has a ground truth file
    bool has_ground_truth(const DatasetOptions &, int sequence_id);

    // Loads the Ground Truth of a given sequence of a Dataset
    ArrayPoses load_ground_truth(const DatasetOptions &, int sequence_id);

    // Loads the Ground Truth in the sensor's reference frame
    ArrayPoses load_sensor_ground_truth(const DatasetOptions &, int sequence_id);

    // Returns a DatasetSequence
    std::shared_ptr<DatasetSequence> get_dataset_sequence(const DatasetOptions &, int sequence_id = -1);

}


#endif //CT_ICP_DATASET_HPP
