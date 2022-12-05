#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include <Eigen/Dense>

#include <SlamCore/types.h>
#include <SlamCore/pointcloud.h>
#include <SlamCore/io.h>
#include <SlamCore/imu.h>
#include <SlamCore/algorithm/grid_sampling.h>

#include <ct_icp/dataset.h>
#include <ct_icp/odometry.h>
#include <ct_icp/config.h>
#include <ct_icp/map.h>

namespace py = pybind11;

std::string SlamTypeToPybind11DTYPEForm(slam::PROPERTY_TYPE type)
{
    switch (type)
    {
    case slam::UINT8:
        return py::format_descriptor<std::uint8_t>::format();
    case slam::INT8:
        return py::format_descriptor<std::int8_t>::format();
    case slam::UINT16:
        return py::format_descriptor<std::uint16_t>::format();
    case slam::INT16:
        return py::format_descriptor<std::int16_t>::format();
    case slam::UINT32:
        return py::format_descriptor<std::uint32_t>::format();
    case slam::INT32:
        return py::format_descriptor<std::int32_t>::format();
    case slam::UINT64:
        return py::format_descriptor<std::uint64_t>::format();
    case slam::INT64:
        return py::format_descriptor<std::int64_t>::format();
    case slam::FLOAT32:
        return py::format_descriptor<float>::format();
    case slam::FLOAT64:
        return py::format_descriptor<double>::format();
    default:
        throw std::runtime_error("Unsupported Type!");
    }
}

// Wrapper for a point cloud
// A point cloud should be interpretable as a numpy structured array (and reciprocally)
class PY_PointCloud
{
public:
    PY_PointCloud()
    {
        pointcloud = slam::PointCloud::DefaultXYZPtr<float>();
    }

    PY_PointCloud(slam::PointCloudPtr pc) : pointcloud(pc) {}

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// API
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    PY_PointCloud Clone() const
    {
        return PY_PointCloud{pointcloud->DeepCopyPtr()};
    }

    void Resize(size_t num_points)
    {
        pointcloud->resize(num_points);
    }

    size_t Size() const
    {
        return pointcloud->size();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// FIELDS GETTERS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /*! @brief  Defines a Getter for a PointCloud Field, and returns a numpy array providing a mutable view of the data
     *          Without any copy (non const method) or with a copy (const method).
     */
#define GETTER_POINTCLOUD_FIELD(field)                                                                  \
    py::array Get##field()                                                                              \
    {                                                                                                   \
        SLAM_CHECK_STREAM(pointcloud, "The point cloud is null !");                                     \
        SLAM_CHECK_STREAM(pointcloud->Has##field(), "The point cloud does not have a " #field "Field"); \
        auto _field = pointcloud->Get##field##Field();                                                  \
        SLAM_CHECK_STREAM(!_field.IsItem(), "The xyz field is an item... This should not happen");      \
        return py::array(BufferInfoFromField(_field), hack);                                            \
    };                                                                                                  \
    py::array Get##field##Copy() const                                                                  \
    {                                                                                                   \
        SLAM_CHECK_STREAM(pointcloud, "The point cloud is null !");                                     \
        SLAM_CHECK_STREAM(pointcloud->Has##field(), "The point cloud does not have a ##field## Field"); \
        auto _field = pointcloud->Get##field##Field();                                                  \
        SLAM_CHECK_STREAM(!_field.IsItem(), "The xyz field is an item... This should not happen");      \
        return py::array(BufferInfoFromField(_field));                                                  \
    };                                                                                                  \
    bool Has##field##Field() const                                                                      \
    {                                                                                                   \
        SLAM_CHECK_STREAM(pointcloud, "The point cloud is null !");                                     \
        return pointcloud->Has##field();                                                                \
    };

    py::array GetXYZ()
    {
        // Special treatment for XYZ, as it does not have a GetXYZField method
        SLAM_CHECK_STREAM(pointcloud, "The point cloud is null !");
        auto &field = pointcloud->GetXYZField();
        SLAM_CHECK_STREAM(!field.IsItem(), "The xyz field is an item... This should not happen");
        return py::array(BufferInfoFromField(field), hack); // Passes the handle to avoid a copy of the array
    };

    py::array GetXYZCopy() const
    {
        // Special treatment for XYZ, as it does not have a GetXYZField method
        SLAM_CHECK_STREAM(pointcloud, "The point cloud is null !");
        auto &field = pointcloud->GetXYZField();
        SLAM_CHECK_STREAM(!field.IsItem(), "The xyz field is an item... This should not happen");
        return py::array(BufferInfoFromField(field));
    };

    GETTER_POINTCLOUD_FIELD(RawPoints)

    GETTER_POINTCLOUD_FIELD(WorldPoints)

    GETTER_POINTCLOUD_FIELD(Normals)

    GETTER_POINTCLOUD_FIELD(RGB)

    GETTER_POINTCLOUD_FIELD(Intensity)

    GETTER_POINTCLOUD_FIELD(Timestamps)

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///  Add default fields
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ADD_DEFAULT_FIELD(field)                                                               \
    void Add##field##Field()                                                                   \
    {                                                                                          \
        SLAM_CHECK_STREAM(pointcloud, "The point cloud is null");                              \
        SLAM_CHECK_STREAM(!pointcloud->Has##field(), "The " #field " Field already exists !"); \
        pointcloud->AddDefault##field##Field();                                                \
    }

    ADD_DEFAULT_FIELD(RawPoints)

    ADD_DEFAULT_FIELD(WorldPoints)

    ADD_DEFAULT_FIELD(RGB)

    ADD_DEFAULT_FIELD(Timestamps)

    ADD_DEFAULT_FIELD(Intensity)

    ADD_DEFAULT_FIELD(Normals)

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// TODO : GENERIC PointCloud Fields API
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    slam::PointCloudPtr pointcloud = nullptr;

private:
    py::str hack; // Handle to prevent unsolicited copies when returning an array

    py::buffer_info BufferInfoFromField(const slam::PointCloud::Field &field) const
    {
        SLAM_CHECK_STREAM(pointcloud, "The point cloud is null !");
        auto buff_info = pointcloud->GetBufferInfoFromField(field);
        return py::buffer_info(
            (void *)buff_info.item_data_ptr,
            ssize_t(slam::PropertySize(buff_info.property_type)),
            SlamTypeToPybind11DTYPEForm(buff_info.property_type),
            ssize_t(2),
            {ssize_t(buff_info.num_items), ssize_t(buff_info.dimensions)},
            {ssize_t(buff_info.item_size), ssize_t(slam::PropertySize(buff_info.property_type))});
    }
};

// struct PyLiDARPoint {
//     double raw_point[3];
//     double pt[3];
//     double alpha_timestamp;
//     double timestamp;
//     int frame_index;
// };
//
//
///// A Wrapper for an array of ct_icp Points
///// Allows access to ct_icp::Point3D from python using numpy's structured dtypes
// struct LiDARFrame {
//
//     std::vector<ct_icp::Point3D> points;
//
//     void SetFrame(const py::array_t<PyLiDARPoint> &arr) {
//         auto req = arr.request();
//         auto size = arr.size();
//         auto ptr = static_cast<ct_icp::Point3D *>(req.ptr);
//
//         points.resize(size);
//         std::copy(ptr, ptr + size, points.begin());
//     };
//
//     py::array_t<PyLiDARPoint> GetWrappingArray(py::handle handle = py::handle()) {
//         py::buffer_info buffer_info(
//                 static_cast<void *>(&points[0]),
//                 sizeof(PyLiDARPoint),
//                 py::format_descriptor<PyLiDARPoint>::format(),
//                 1,
//                 {points.size()},
//                 {sizeof(PyLiDARPoint)});
//
//         py::array_t<PyLiDARPoint> array(buffer_info, handle);
//         return array;
//     }
// };
//
////! Wraps a RegistrationSummary (in order to wrap the std::vector of points with a LiDARFrame)
// struct PyRegistrationSummary : ct_icp::Odometry::RegistrationSummary {
//
//     explicit PyRegistrationSummary(ct_icp::Odometry::RegistrationSummary &&summary_)
//             : ct_icp::Odometry::RegistrationSummary(std::move(summary_)) {
//         lidar_points.points = std::move(corrected_points);
//     }
//
//     LiDARFrame lidar_points;
// };
//
#define STRUCT_READWRITE(_struct, argument) .def_readwrite(#argument, &_struct ::argument)
#define ADD_VALUE(_enum, _value) .value(#_value, _enum ::_value)

//// Copies a vector to a np_array
// template<typename T, typename Scalar = T, typename Alloc_ = std::allocator<T>>
// py::array_t<Scalar> vector_to_ndarray(const std::vector<T, Alloc_> &vector) {
//
//     const auto width = static_cast<py::ssize_t >(sizeof(T) / sizeof(Scalar));
//     py::ssize_t ndim;
//     std::vector<size_t> shape, strides;
//     if (width == 1) {
//         ndim = 1;
//         shape = {vector.size()};
//         strides = {sizeof(T)};
//     } else {
//         ndim = 2;
//         shape = {vector.size(), width};
//         strides = {sizeof(Scalar) * width, sizeof(Scalar)};
//     }
//
//     py::array_t<Scalar> result(py::buffer_info(
//     (void *) (&vector[0]), static_cast<py::ssize_t>(sizeof(Scalar)),
//             py::format_descriptor<Scalar>::format(),
//             ndim,
//             shape,
//             strides
//     ));
//
//     return result;
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// CT_ICP - TRAMPOLINE CLASSES
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*! Trampoline class to define python class extending a dataset sequence */
class PyDatasetSequence : public ct_icp::ADatasetSequence
{
public:
    using ct_icp::ADatasetSequence::ADatasetSequence;

    void SetInitFrame(int frame_index) override
    {
        PYBIND11_OVERRIDE_PURE(void, ct_icp::ADatasetSequence, SetInitFrame, frame_index);
    };

    Frame NextUnfilteredFrame() override
    {
        PYBIND11_OVERRIDE_PURE(Frame, ct_icp::ADatasetSequence, NextUnfilteredFrame);
    };

    size_t NumFrames() const override
    {
        PYBIND11_OVERRIDE_PURE(size_t, ct_icp::ADatasetSequence, NumFrames);
    }

    bool HasNext() const override
    {
        PYBIND11_OVERRIDE(bool, ct_icp::ADatasetSequence, HasNext);
    }
};

class PyIMapOptions : public ct_icp::IMapOptions
{
public:
    std::string GetType() const override
    {
        PYBIND11_OVERRIDE(std::string, ct_icp::IMapOptions, GetType);
    }

    std::shared_ptr<ct_icp::ISlamMap> MakeMapFromOptions() const override
    {
        PYBIND11_OVERRIDE(std::shared_ptr<ct_icp::ISlamMap>, ct_icp::IMapOptions, MakeMapFromOptions);
    }
};

PYBIND11_MODULE(pyct_icp, m)
{

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// BASIC TYPES
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    py::class_<slam::SE3>(m, "SE3")
        .def(py::init())
        .def_readwrite("tr", &slam::SE3::tr)
        .def("Rotation", &slam::SE3::Rotation)
        .def("Matrix", &slam::SE3::Matrix)
        .def("Inverse", &slam::SE3::Inverse)
        .def("Interpolate", [](const slam::SE3 &self, const slam::SE3 &other, double alpha_timestamp)
             { return self.Interpolate(other, alpha_timestamp); })
        .def_static("Random", &slam::SE3::Random, "Generates a Random pose with optional scale parameters",
                    py::arg("tr_scale") = 1., py::arg("rot_scale") = 1.)
        .def("Parameters", &slam::SE3::Parameters)
        .def(py::self * py::self)
        .def(py::self * Eigen::Vector3d())
        .def("__repr__", [](slam::SE3 &pose)
             {
                std::stringstream ss_repr;
                ss_repr << "{ tr: [" << pose.tr.x() << ", " << pose.tr.y() << ", " << pose.tr.z() << "], "
                        << " quat: [" << pose.quat.coeffs()[0] << "," << pose.quat.coeffs()[1] << ","
                        << pose.quat.coeffs()[2] << "," << pose.quat.coeffs()[2] << "] }";
                return ss_repr.str(); })
        .def("SetRotation", [](slam::SE3 &pose, Eigen::Matrix3d rot)
             { pose.quat = Eigen::Quaterniond(rot); });

    m.def("AngularDistance",
          [](const slam::SE3 &lhs, const slam::SE3 &rhs)
          { return slam::AngularDistance(lhs, rhs); });
    m.def("AngularDistanceMat",
          [](const Eigen::Matrix3d &lhs, const Eigen::Matrix3d &rhs)
          { return slam::AngularDistance(lhs, rhs); });

    py::class_<slam::Pose>(m, "Pose")
        .def(py::init())
        .def_readwrite("pose", &slam::Pose::pose)
        .def_readwrite("dest_timestamp", &slam::Pose::dest_timestamp)
        .def_readwrite("dest_frame_id", &slam::Pose::dest_frame_id)
        .def("Matrix", &slam::Pose::Matrix)
        .def("Inverse", &slam::Pose::Inverse)
        .def(py::self * py::self)
        .def(py::self * Eigen::Vector3d())
        .def("ContinuousTransform", &slam::Pose::ContinuousTransform)
        .def("InterpolatePose", &slam::Pose::InterpolatePose,
             "Interpolates a pose at a timestamp between two poses",
             py::arg("other_pose"), py::arg("timestamp"), py::arg("dest_frame_id") = -1)
        .def_static("Identity", []
                    { return slam::Pose::Identity(); });

    py::class_<slam::ImuData>(m, "ImuData")
        .def(py::init())
        .def_readwrite("angular_velocity", &slam::ImuData::angular_velocity)
        .def_readwrite("linear_acceleration", &slam::ImuData::linear_acceleration)
        .def_readwrite("orientation", &slam::ImuData::orientation);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// POINT CLOUD API
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MODULE_PC_FIELD(field) \
    .def("Get" #field, &PY_PointCloud::Get##field)
#define MODULE_PC_FIELD_WITH_DEFAULT(field)                           \
    MODULE_PC_FIELD(field)                                            \
        .def("Add" #field "Field", &PY_PointCloud::Add##field##Field) \
        .def("Has" #field "Field", &PY_PointCloud::Has##field##Field)

    py::class_<PY_PointCloud>(m, "PointCloud")
        .def(py::init())
        .def("SetRawPointsFromXYZ", [](PY_PointCloud &pc)
             {
            SLAM_CHECK_STREAM((pc.pointcloud != nullptr), "The point cloud is not null !");
            pc.pointcloud->SetRawPointsField(slam::PointCloud::Field(pc.pointcloud->GetXYZField())); })
            MODULE_PC_FIELD(XYZ)
                MODULE_PC_FIELD_WITH_DEFAULT(RawPoints)
                    MODULE_PC_FIELD_WITH_DEFAULT(WorldPoints)
                        MODULE_PC_FIELD_WITH_DEFAULT(Normals)
                            MODULE_PC_FIELD_WITH_DEFAULT(Intensity)
                                MODULE_PC_FIELD_WITH_DEFAULT(RGB)
                                    MODULE_PC_FIELD_WITH_DEFAULT(Timestamps)
        .def("Size", &PY_PointCloud::Size)
        .def("Resize", &PY_PointCloud::Resize)
        .def("Clone", &PY_PointCloud::Clone);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IO Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    m.def("WritePointCloudAsPLY", [](const std::string &filepath, const PY_PointCloud &pc)
          {
        SLAM_CHECK_STREAM(pc.pointcloud, "The point cloud is null")
        try {
            slam::WritePLY(filepath, *pc.pointcloud,
                           slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(pc.pointcloud->GetCollection()));
        } catch (std::exception &e) {
            SLAM_LOG(ERROR) << "Failed to write the point cloud to the file " << filepath << std::endl;
            SLAM_LOG(ERROR) << "Caught exception " << e.what() << std::endl;
        } });

    m.def("ReadPointCloudFromPLY", [](const std::string &filepath)
          {
        auto pc = slam::ReadPointCloudFromPLY(filepath);
        pc->RegisterFieldsFromSchema();
        return PY_PointCloud{pc}; });

    m.def("ReadPosesFromPLY", [](const std::string &filepath)
          { return slam::ReadPosesFromPLY(filepath); });
    m.def("WritePosesAsPLY", [](const std::string &filepath,
                                const std::vector<slam::Pose> &poses)
          { slam::SavePosesAsPLY(filepath, poses); });
    m.def("ReadKITTIPoses", [](const std::string &filepath)
          { slam::LoadPosesKITTIFormat(filepath); });
    m.def("SaveKITTIPoses", [](const std::string &filepath,
                               const std::vector<slam::Pose> &poses)
          { slam::SavePosesKITTIFormat(filepath, poses); });

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// CT_ICP - DATASETS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    py::enum_<ct_icp::DATASET>(m, "DATASET")
        .value("KITTI_raw", ct_icp::DATASET::KITTI_raw)
        .value("KITTI_CARLA", ct_icp::DATASET::KITTI_CARLA)
        .value("KITTI", ct_icp::DATASET::KITTI)
        .value("KITTI_360", ct_icp::DATASET::KITTI_360)
        .value("NCLT", ct_icp::DATASET::NCLT)
        .value("HILTI_2021", ct_icp::DATASET::HILTI_2021)
        .value("HILTI_2022", ct_icp::DATASET::HILTI_2022)
        .value("PLY_DIRECTORY", ct_icp::DATASET::PLY_DIRECTORY)
        .value("SYNTHETIC", ct_icp::DATASET::SYNTHETIC)
        .value("CUSTOM", ct_icp::DATASET::CUSTOM)
        .value("INVALID", ct_icp::DATASET::INVALID);

    m.def("DATASETFromString", &ct_icp::DATASETFromString);
    m.def("DATASETEnumToString", &ct_icp::DATASETEnumToString);

    py::class_<ct_icp::SequenceInfo>(m, "SequenceInfo")
        .def_readwrite("sequence_name", &ct_icp::SequenceInfo::sequence_name)
        .def_readwrite("label", &ct_icp::SequenceInfo::label)
        .def_readwrite("sequence_id", &ct_icp::SequenceInfo::sequence_id)
        .def_readwrite("sequence_size", &ct_icp::SequenceInfo::sequence_size)
        .def_readwrite("with_ground_truth", &ct_icp::SequenceInfo::with_ground_truth);

    py::class_<ct_icp::LidarIMUFrame, ct_icp::LidarIMUFramePtr>(m, "LidarIMUFrame")
        .def(py::init())
        .def_property(
            "pointcloud", [](const ct_icp::LidarIMUFrame &frame)
            { return PY_PointCloud{frame.pointcloud}; },
            [](ct_icp::LidarIMUFrame &frame, const PY_PointCloud &pc)
            {
                frame.pointcloud = pc.pointcloud;
            })
        .def_readwrite("timestamp_min", &ct_icp::LidarIMUFrame::timestamp_min)
        .def_readwrite("timestamp_max", &ct_icp::LidarIMUFrame::timestamp_max)
        .def_readwrite("imu_data", &ct_icp::LidarIMUFrame::imu_data)
        .def_readwrite("file_path", &ct_icp::LidarIMUFrame::file_path);

    py::class_<ct_icp::ADatasetSequence, PyDatasetSequence, std::shared_ptr<ct_icp::ADatasetSequence>>(m,
                                                                                                       "DatasetSequence")
        .def("HasNext", &ct_icp::ADatasetSequence::HasNext)
        .def("NextFrame", &ct_icp::ADatasetSequence::NextFrame)
        .def("NumFrames", &ct_icp::ADatasetSequence::NumFrames)
        .def("WithRandomAccess", &ct_icp::ADatasetSequence::WithRandomAccess)
        .def("GetFrame", &ct_icp::ADatasetSequence::GetFrame)
        .def("HasGroundTruth", &ct_icp::ADatasetSequence::HasGroundTruth)
        .def("SkipFrame", &ct_icp::ADatasetSequence::SkipFrame)
        .def("GetSequenceName", [](const ct_icp::ADatasetSequence& seq) {
            return seq.GetSequenceInfo().sequence_name;
        });

    py::class_<ct_icp::AFileSequence, ct_icp::ADatasetSequence, std::shared_ptr<ct_icp::AFileSequence>>(m,
                                                                                                        "FileSequence")
        .def("HasNext", &ct_icp::AFileSequence::HasNext)
        .def("NextUnfilteredFrame", &ct_icp::AFileSequence::NextUnfilteredFrame)
        .def("NumFrames", &ct_icp::AFileSequence::NumFrames)
        .def("GetFilePaths", &ct_icp::AFileSequence::GetFilePaths)
        .def("SkipFrame", &ct_icp::AFileSequence::SkipFrame);

    py::class_<ct_icp::PLYDirectory, ct_icp::AFileSequence, std::shared_ptr<ct_icp::PLYDirectory>>(m, "PLYDirectory")
        .def(py::init([](const std::string &root_path)
                      {
                auto directory = ct_icp::PLYDirectory::PtrFromDirectoryPath(root_path);
                return directory; }))
        .def("ReadFrame", &ct_icp::PLYDirectory::ReadFrame);

    py::class_<ct_icp::NCLTIterator, ct_icp::ADatasetSequence, std::shared_ptr<ct_icp::NCLTIterator>>(m, "NCLTIterator")
        .def(py::init([](const std::string &hits_file, const std::string &sequence_name)
                      {
                auto directory = ct_icp::NCLTIterator::NCLTIteratorFromHitsFile(hits_file, sequence_name);
                return directory; }))
        .def("DoNext", &ct_icp::NCLTIterator::DoNext);

    py::class_<ct_icp::DatasetOptions>(m, "DatasetOptions")
        .def(py::init())
        .def_readwrite("dataset", &ct_icp::DatasetOptions::dataset)
        .def_readwrite("root_path", &ct_icp::DatasetOptions::root_path)
        .def_readwrite("fail_if_incomplete", &ct_icp::DatasetOptions::fail_if_incomplete)
        .def_readwrite("min_dist_lidar_center", &ct_icp::DatasetOptions::min_dist_lidar_center)
        .def_readwrite("max_dist_lidar_center", &ct_icp::DatasetOptions::max_dist_lidar_center)
        .def_readwrite("nclt_num_aggregated_pc", &ct_icp::DatasetOptions::nclt_num_aggregated_pc);

    m.def("DatasetOptionsFromYAMLStr", [](const std::string& yaml_str) {
        YAML::Node node = YAML::Load(yaml_str);
        return ct_icp::yaml_to_dataset_options(node);
    });

    m.def("DatasetOptionsFromYAMLFile", [](const std::string& yaml_file) {
        YAML::Node node = YAML::LoadFile(yaml_file);
        return ct_icp::yaml_to_dataset_options(node);
    });

    py::class_<ct_icp::Dataset>(m, "Dataset")
        .def(py::init([](const ct_icp::DatasetOptions& options){
            return ct_icp::Dataset::LoadDataset(options);
        }))
        .def(py::init([](const std::vector<std::shared_ptr<ct_icp::ADatasetSequence>>& sequences) {
            return ct_icp::Dataset::BuildCustomDataset(std::vector<std::shared_ptr<ct_icp::ADatasetSequence>>(sequences));
        }))
        .def("AllSequences", &ct_icp::Dataset::AllSequences)
        .def("HasSequence", &ct_icp::Dataset::HasSequence)
        .def("HasGroundTruth", &ct_icp::Dataset::HasGroundTruth)
        .def("GetSequence", &ct_icp::Dataset::GetSequence)
        .def("DatasetType", &ct_icp::Dataset::DatasetType)
        .def("GetGroundTruth", &ct_icp::Dataset::GetGroundTruth);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// CT_ICP Map
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    py::class_<ct_icp::ISlamMap, std::shared_ptr<ct_icp::ISlamMap>>(m, "ISlamMap")
        .def("InsertPointCloud", [](ct_icp::ISlamMap &map,
                                    const PY_PointCloud &pc,
                                    const std::vector<slam::Pose> &frame_poses)
             {
                SLAM_CHECK_STREAM(pc.pointcloud, "[PY_PointCloud] The point cloud is null !");
                map.InsertPointCloud(*pc.pointcloud, frame_poses); })
        .def("MapAsPointCloud", [](const ct_icp::ISlamMap &map)
             { return PY_PointCloud{map.MapAsPointCloud()}; })
        .def("NumPoints", &ct_icp::ISlamMap::NumPoints)
        .def("RadiusSearch",
             &ct_icp::ISlamMap::RadiusSearch,
             "Returns the neighborhood for a query point", py::arg("query"), py::arg("radius"),
             py::arg("max_num_neighbors") = -1, py::arg("neares_neighbors") = true,
             py::arg("sensor_location") = nullptr);

    py::class_<ct_icp::MultipleResolutionVoxelMap::SearchParams>(m, "Map_SearchParams")
        .def(py::init())
            STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::SearchParams, radius)
                STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::SearchParams, voxel_resolution)
                    STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::SearchParams, map_id)
                        STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::SearchParams, voxel_neighborhood);

    py::class_<ct_icp::MultipleResolutionVoxelMap::ResolutionParam>(m, "Map_ResolutionParam")
        .def(py::init())
            STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::ResolutionParam, resolution)
                STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::ResolutionParam, min_distance_between_points)
                    STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::ResolutionParam, max_num_points);

    py::class_<ct_icp::IMapOptions, PyIMapOptions, std::shared_ptr<ct_icp::IMapOptions>>(m, "IMapOptions")
        .def("GetType", &ct_icp::IMapOptions::GetType)
        .def("MakeMapFromOptions", &ct_icp::IMapOptions::MakeMapFromOptions);

    py::class_<ct_icp::MultipleResolutionVoxelMap::Options, ct_icp::IMapOptions, std::shared_ptr<ct_icp::MultipleResolutionVoxelMap::Options>>(m, "Map_Options")
        .def(py::init())
        .def(py::init([](const std::string &yaml_str)
                      {
                YAML::Node node = YAML::Load(yaml_str);
                auto map_options = ct_icp::yaml_to_map_options(node);
                auto cast_options = std::dynamic_pointer_cast<ct_icp::MultipleResolutionVoxelMap::Options>(map_options);
                SLAM_CHECK_STREAM(cast_options, "Could not create Map Options from the following yaml:\n"
                        << node << std::endl;);
                return *cast_options; }))
            STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::Options, resolutions)
                STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::Options, select_valid_normals_direction)
                    STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::Options, max_frames_to_keep)
                        STRUCT_READWRITE(ct_icp::MultipleResolutionVoxelMap::Options, default_radius);

    py::class_<ct_icp::MultipleResolutionVoxelMap, ct_icp::ISlamMap,
               std::shared_ptr<ct_icp::MultipleResolutionVoxelMap>>(m, "MultipleResolutionVoxelMap")
        .def(py::init())
        .def(py::init([](const ct_icp::MultipleResolutionVoxelMap::Options &options)
                      { return std::make_shared<ct_icp::MultipleResolutionVoxelMap>(options); }))
        .def("RemoveElementsFarFromLocation",
             &ct_icp::MultipleResolutionVoxelMap::RemoveElementsFarFromLocation)
        .def(
            "GetMapPoints", [](const ct_icp::MultipleResolutionVoxelMap &map, int map_idx)
            { return PY_PointCloud{map.GetMapPoints(map_idx)}; },
            "Returns the Map Points for the map", py::arg("map_idx") = 0)
        .def("GetVisibleMapPoints", &ct_icp::MultipleResolutionVoxelMap::GetVisibleMapPoints);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// CT_ICP Registration
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    py::enum_<ct_icp::CT_ICP_SOLVER>(m, "CT_ICP_SOLVER")
        ADD_VALUE(ct_icp::CT_ICP_SOLVER, CERES)
            ADD_VALUE(ct_icp::CT_ICP_SOLVER, GN)
                ADD_VALUE(ct_icp::CT_ICP_SOLVER, ROBUST)
                    .export_values();

    py::enum_<ct_icp::LEAST_SQUARES>(m, "LEAST_SQUARES")
        ADD_VALUE(ct_icp::LEAST_SQUARES, STANDARD)
            ADD_VALUE(ct_icp::LEAST_SQUARES, CAUCHY)
                ADD_VALUE(ct_icp::LEAST_SQUARES, HUBER)
                    ADD_VALUE(ct_icp::LEAST_SQUARES, TOLERANT)
                        ADD_VALUE(ct_icp::LEAST_SQUARES, TRUNCATED)
                            .export_values();

    py::enum_<ct_icp::WEIGHTING_SCHEME>(m, "WEIGHTING_SCHEME")
        ADD_VALUE(ct_icp::WEIGHTING_SCHEME, PLANARITY)
            ADD_VALUE(ct_icp::WEIGHTING_SCHEME, NEIGHBORHOOD)
                ADD_VALUE(ct_icp::WEIGHTING_SCHEME, ALL)
                    .export_values();

    py::enum_<ct_icp::POSE_PARAMETRIZATION>(m, "POSE_PARAMETRIZATION")
        ADD_VALUE(ct_icp::POSE_PARAMETRIZATION, SIMPLE)
            ADD_VALUE(ct_icp::POSE_PARAMETRIZATION, CONTINUOUS_TIME)
                .export_values();

    py::enum_<ct_icp::ICP_DISTANCE>(m, "ICP_DISTANCE")
        ADD_VALUE(ct_icp::ICP_DISTANCE, POINT_TO_PLANE)
            ADD_VALUE(ct_icp::ICP_DISTANCE, POINT_TO_POINT)
                ADD_VALUE(ct_icp::ICP_DISTANCE, POINT_TO_LINE)
                    ADD_VALUE(ct_icp::ICP_DISTANCE, POINT_TO_DISTRIBUTION)
                        .export_values();

    py::class_<ct_icp::CTICPOptions,
               std::shared_ptr<ct_icp::CTICPOptions>>(m, "CTICPOptions")
        .def(py::init())
        // ----- Main Params
        STRUCT_READWRITE(ct_icp::CTICPOptions, num_iters_icp)
            STRUCT_READWRITE(ct_icp::CTICPOptions, parametrization)
                STRUCT_READWRITE(ct_icp::CTICPOptions, distance)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, solver)
        // ----- Robustness scheme
        STRUCT_READWRITE(ct_icp::CTICPOptions, max_num_residuals)
            STRUCT_READWRITE(ct_icp::CTICPOptions, min_num_residuals)
                STRUCT_READWRITE(ct_icp::CTICPOptions, weighting_scheme)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, weight_alpha)
                        STRUCT_READWRITE(ct_icp::CTICPOptions, weight_neighborhood)
        // ----- Neighborhood Params
        STRUCT_READWRITE(ct_icp::CTICPOptions, power_planarity)
            STRUCT_READWRITE(ct_icp::CTICPOptions, max_number_neighbors)
                STRUCT_READWRITE(ct_icp::CTICPOptions, min_number_neighbors)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, threshold_voxel_occupancy)
                        STRUCT_READWRITE(ct_icp::CTICPOptions, threshold_voxel_occupancy)
                            STRUCT_READWRITE(ct_icp::CTICPOptions, estimate_normal_from_neighborhood)
                                STRUCT_READWRITE(ct_icp::CTICPOptions, num_closest_neighbors)
        // ----- Stop criterion params
        STRUCT_READWRITE(ct_icp::CTICPOptions, threshold_orientation_norm)
            STRUCT_READWRITE(ct_icp::CTICPOptions, threshold_translation_norm)
        // ----- Continuous Time Trajectory Constraint Params
        STRUCT_READWRITE(ct_icp::CTICPOptions, point_to_plane_with_distortion)
        // ----- CERES Solver Specific params
        STRUCT_READWRITE(ct_icp::CTICPOptions, loss_function)
            STRUCT_READWRITE(ct_icp::CTICPOptions, ls_max_num_iters)
                STRUCT_READWRITE(ct_icp::CTICPOptions, ls_num_threads)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, ls_sigma)
                        STRUCT_READWRITE(ct_icp::CTICPOptions, ls_tolerant_min_threshold)
        // ----- GN Solver Specific params
        STRUCT_READWRITE(ct_icp::CTICPOptions, max_dist_to_plane_ct_icp)
        // ----- ROBUST Solver params
        STRUCT_READWRITE(ct_icp::CTICPOptions, threshold_linearity)
            STRUCT_READWRITE(ct_icp::CTICPOptions, threshold_planarity)
                STRUCT_READWRITE(ct_icp::CTICPOptions, weight_point_to_point)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, outlier_distance)
                        STRUCT_READWRITE(ct_icp::CTICPOptions, use_barycenter)
                            STRUCT_READWRITE(ct_icp::CTICPOptions, use_lines)
                                STRUCT_READWRITE(ct_icp::CTICPOptions, use_distribution)
        // ----- OUTPUT /DEBUG  params
        STRUCT_READWRITE(ct_icp::CTICPOptions, output_residuals)
            STRUCT_READWRITE(ct_icp::CTICPOptions, output_weights)
                STRUCT_READWRITE(ct_icp::CTICPOptions, output_normals)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, output_lines)
                        STRUCT_READWRITE(ct_icp::CTICPOptions, debug_print);

    py::class_<ct_icp::ICPSummary>(m, "ICPSummary")
        .def(py::init())
        .def_readwrite("success", &ct_icp::ICPSummary::success)
        .def_readwrite("num_residuals_used", &ct_icp::ICPSummary::num_residuals_used)
        .def_readwrite("num_iters", &ct_icp::ICPSummary::num_iters)
        .def_readwrite("error_log", &ct_icp::ICPSummary::error_log)
        .def_readwrite("duration_total", &ct_icp::ICPSummary::duration_total)
        .def_readwrite("duration_init", &ct_icp::ICPSummary::duration_init)
        .def_readwrite("avg_duration_iter", &ct_icp::ICPSummary::avg_duration_iter)
        .def_readwrite("avg_duration_neighborhood", &ct_icp::ICPSummary::avg_duration_neighborhood)
        .def_readwrite("avg_duration_solve", &ct_icp::ICPSummary::avg_duration_solve);

    m.def("CTICPOptionsFromYAMLStr", [](const std::string &yaml_str)
          {
        YAML::Node node = YAML::Load(yaml_str);
        return ct_icp::yaml_to_ct_icp_options(node); });

    m.def("CTICPOptionsFromYAMLFile", [](const std::string &yaml_file)
          {
        YAML::Node node = YAML::LoadFile(yaml_file);
        return ct_icp::yaml_to_ct_icp_options(node); });

    py::class_<ct_icp::TrajectoryFrame>(m, "TrajectoryFrame")
        .def(py::init())
        .def_readwrite("begin_pose", &ct_icp::TrajectoryFrame::begin_pose)
        .def_readwrite("end_pose", &ct_icp::TrajectoryFrame::end_pose)
        .def("MidPose", &ct_icp::TrajectoryFrame::MidPose);

    py::class_<ct_icp::CT_ICP_Registration>(m, "CTICP_Registration")
        .def(py::init())
        .def(py::init([](const ct_icp::CTICPOptions &options)
                      {
        ct_icp::CT_ICP_Registration icp;
        icp.Options() = options;
        return icp; }))
        .def("SetOptions", [](ct_icp::CT_ICP_Registration &icp, const ct_icp::CTICPOptions &options)
             { icp.Options() = options; })
        .def("GetOptions", [](const ct_icp::CT_ICP_Registration &icp)
             { return icp.Options(); })
        .def("Register", [](ct_icp::CT_ICP_Registration &icp, const ct_icp::ISlamMap &map, PY_PointCloud &cloud, ct_icp::TrajectoryFrame &initial_frame)
             { return icp.Register(map, *cloud.pointcloud, initial_frame, nullptr, nullptr); });

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// CT_ICP Sampling
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    py::class_<slam::GridSamplingOptions>(m, "GridSamplingOptions")
        .def(py::init())
            STRUCT_READWRITE(slam::GridSamplingOptions, grid_size)
                STRUCT_READWRITE(slam::GridSamplingOptions, num_points_per_voxel)
                    STRUCT_READWRITE(slam::GridSamplingOptions, max_num_points);

    m.def("SamplePointCloudInGrid", [](const PY_PointCloud &pc, const slam::GridSamplingOptions &options)
          { return PY_PointCloud{slam::SamplePointCloudInGrid(*pc.pointcloud, options)}; });

    py::class_<ct_icp::AdaptiveGridSamplingOptions>(m, "AdaptiveGridSamplingOptions")
        .def(py::init())
            STRUCT_READWRITE(ct_icp::AdaptiveGridSamplingOptions, num_points_per_voxel)
                STRUCT_READWRITE(ct_icp::AdaptiveGridSamplingOptions, max_num_points)
                    STRUCT_READWRITE(ct_icp::AdaptiveGridSamplingOptions, distance_voxel_size);

    m.def("AdaptivePointCloudInGrid", [](const PY_PointCloud &pc,
                                         const ct_icp::AdaptiveGridSamplingOptions &options)
          { return PY_PointCloud{
                ct_icp::AdaptiveSamplePointCloudInGrid(*pc.pointcloud, options)}; });

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// CT_ICP Odometry
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    py::enum_<ct_icp::MOTION_COMPENSATION>(m, "MOTION_COMPENSATION")
        ADD_VALUE(ct_icp::MOTION_COMPENSATION, NONE)
            ADD_VALUE(ct_icp::MOTION_COMPENSATION, CONSTANT_VELOCITY)
                ADD_VALUE(ct_icp::MOTION_COMPENSATION, ITERATIVE)
                    ADD_VALUE(ct_icp::MOTION_COMPENSATION, CONTINUOUS)
                        .export_values();

    py::enum_<ct_icp::INITIALIZATION>(m, "INITIALIZATION")
        ADD_VALUE(ct_icp::INITIALIZATION, INIT_NONE)
            ADD_VALUE(ct_icp::INITIALIZATION, INIT_CONSTANT_VELOCITY)
                .export_values();

    py::enum_<ct_icp::sampling::SAMPLING_OPTION>(m, "SAMPLING_OPTION")
        ADD_VALUE(ct_icp::sampling::SAMPLING_OPTION, NONE)
            ADD_VALUE(ct_icp::sampling::SAMPLING_OPTION, GRID)
                ADD_VALUE(ct_icp::sampling::SAMPLING_OPTION, ADAPTIVE)
                    .export_values();

    py::class_<ct_icp::OdometryOptions>(m, "OdometryOptions")
        .def(py::init())
        .def_static("DefaultDrivingProfile", ct_icp::OdometryOptions::DefaultDrivingProfile)
        .def_static("DefaultRobustOutdoorLowInertia", ct_icp::OdometryOptions::DefaultRobustOutdoorLowInertia)
            STRUCT_READWRITE(ct_icp::OdometryOptions, ct_icp_options)
                STRUCT_READWRITE(ct_icp::OdometryOptions, motion_compensation)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, initialization)
                        STRUCT_READWRITE(ct_icp::OdometryOptions, init_voxel_size)
                            STRUCT_READWRITE(ct_icp::OdometryOptions, init_sample_voxel_size)
                                STRUCT_READWRITE(ct_icp::OdometryOptions, init_num_frames)
                                    STRUCT_READWRITE(ct_icp::OdometryOptions, sample_voxel_size)
                                        STRUCT_READWRITE(ct_icp::OdometryOptions, max_num_keypoints)
                                            STRUCT_READWRITE(ct_icp::OdometryOptions, sampling)
                                                STRUCT_READWRITE(ct_icp::OdometryOptions, map_options)
                                                    STRUCT_READWRITE(ct_icp::OdometryOptions, neighborhood_strategy)
                                                        STRUCT_READWRITE(ct_icp::OdometryOptions, size_voxel_map)
                                                            STRUCT_READWRITE(ct_icp::OdometryOptions, max_num_points_in_voxel)
                                                                STRUCT_READWRITE(ct_icp::OdometryOptions, voxel_neighborhood)
                                                                    STRUCT_READWRITE(ct_icp::OdometryOptions, max_radius_neighborhood)
                                                                        STRUCT_READWRITE(ct_icp::OdometryOptions, min_distance_points)
                                                                            STRUCT_READWRITE(ct_icp::OdometryOptions, voxel_size)
                                                                                STRUCT_READWRITE(ct_icp::OdometryOptions, max_distance)
                                                                                    STRUCT_READWRITE(ct_icp::OdometryOptions, distance_error_threshold)
                                                                                        STRUCT_READWRITE(ct_icp::OdometryOptions, orientation_error_threshold)
                                                                                            STRUCT_READWRITE(ct_icp::OdometryOptions, quit_on_error)
                                                                                                STRUCT_READWRITE(ct_icp::OdometryOptions, always_insert)
                                                                                                    STRUCT_READWRITE(ct_icp::OdometryOptions, do_no_insert)
                                                                                                        STRUCT_READWRITE(ct_icp::OdometryOptions, debug_print)
                                                                                                            STRUCT_READWRITE(ct_icp::OdometryOptions, debug_viz)
                                                                                                                STRUCT_READWRITE(ct_icp::OdometryOptions, log_to_file)
                                                                                                                    STRUCT_READWRITE(ct_icp::OdometryOptions, log_file_destination)
                                                                                                                        STRUCT_READWRITE(ct_icp::OdometryOptions, default_motion_model)
                                                                                                                            STRUCT_READWRITE(ct_icp::OdometryOptions, with_default_motion_model)
                                                                                                                                STRUCT_READWRITE(ct_icp::OdometryOptions, adaptive_options);

    m.def("OdometryOptionsFromYAMLStr", [](const std::string &yaml_str)
          {
        YAML::Node node = YAML::Load(yaml_str);
        return ct_icp::yaml_to_odometry_options(node); });

    m.def("OdometryOptionsFromYAMLFile", [](const std::string &yaml_file)
          {
        YAML::Node node = YAML::LoadFile(yaml_file);
        return ct_icp::yaml_to_odometry_options(node); });

    py::class_<ct_icp::Odometry::RegistrationSummary>(m, "Odometry_RegistrationSummary")
        .def(py::init())
            STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, frame)
                STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, initial_frame)
                    STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, sample_size)
                        STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, number_of_residuals)
                            STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, robust_level)
                                STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, distance_correction)
                                    STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, relative_distance)
                                        STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, relative_orientation)
                                            STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, ego_orientation)
                                                STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, success)
                                                    STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, points_added)
                                                        STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, number_of_attempts)
                                                            STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, error_message)
                                                                STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, icp_summary)
                                                                    STRUCT_READWRITE(ct_icp::Odometry::RegistrationSummary, logged_values);

    py::class_<ct_icp::Odometry, std::shared_ptr<ct_icp::Odometry>>(m, "Odometry")
        .def(py::init([](const ct_icp::OdometryOptions &options)
                      { return std::make_shared<ct_icp::Odometry>(options); }))
        .def("RegisterFrame", [](ct_icp::Odometry &odometry, const PY_PointCloud &pc, int frame_id)
             { return odometry.RegisterFrame(*pc.pointcloud, frame_id); })
        .def("GetTrajectory", &ct_icp::Odometry::Trajectory)
        .def("GetMapPointCloud", [](const ct_icp::Odometry &odom)
             { return odom.GetMapPointCloud(); })
        .def("GetMapSize", &ct_icp::Odometry::MapSize)
        .def("GetMap", &ct_icp::Odometry::GetMapPointer)
        .def("Reset", [](ct_icp::Odometry &odom)
             { odom.Reset(); })
        .def("ResetWithOptions", [](ct_icp::Odometry &odom, const ct_icp::OdometryOptions &options)
             { odom.Reset(options); });
}