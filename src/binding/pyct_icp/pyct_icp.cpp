#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <types.hpp>
#include <iostream>

#include "ct_icp.hpp"
#include "odometry.hpp"
#include "dataset.hpp"

namespace py = pybind11;


struct PyLiDARPoint {
    double raw_point[3];
    double pt[3];
    double alpha_timestamp;
    double timestamp;
    int frame_index;
};


/// A Wrapper for an array of ct_icp Points
/// Allows access to ct_icp::Point3D from python using numpy's structured dtypes
struct LiDARFrame {

    std::vector<ct_icp::Point3D> points;

    void SetFrame(const py::array_t<PyLiDARPoint> &arr) {
        auto req = arr.request();
        auto size = arr.size();
        auto ptr = static_cast<ct_icp::Point3D *>(req.ptr);

        points.resize(size);
        std::copy(ptr, ptr + size, points.begin());
    };

    py::array_t<PyLiDARPoint> GetWrappingArray(py::handle handle = py::handle()) {
        py::buffer_info buffer_info(
                static_cast<void *>(&points[0]),
                sizeof(PyLiDARPoint),
                py::format_descriptor<PyLiDARPoint>::format(),
                1,
                {points.size()},
                {sizeof(PyLiDARPoint)});

        py::array_t<PyLiDARPoint> array(buffer_info, handle);
        return array;
    }
};

//! Wraps a RegistrationSummary (in order to wrap the std::vector of points with a LiDARFrame)
struct PyRegistrationSummary : ct_icp::Odometry::RegistrationSummary {

    explicit PyRegistrationSummary(ct_icp::Odometry::RegistrationSummary &&summary_)
            : ct_icp::Odometry::RegistrationSummary(std::move(summary_)) {
        lidar_points.points = std::move(corrected_points);
    }

    LiDARFrame lidar_points;
};

#define STRUCT_READWRITE(_struct, argument) .def_readwrite(#argument, & _struct :: argument )

#define ADD_VALUE(_enum, _value) .value(#_value, _enum :: _value )

// Copies a vector to a np_array
template<typename T, typename Scalar = T, typename Alloc_ = std::allocator<T>>
py::array_t<Scalar> vector_to_ndarray(const std::vector<T, Alloc_> &vector) {

    const auto width = static_cast<py::ssize_t >(sizeof(T) / sizeof(Scalar));
    py::ssize_t ndim;
    std::vector<size_t> shape, strides;
    if (width == 1) {
        ndim = 1;
        shape = {vector.size()};
        strides = {sizeof(T)};
    } else {
        ndim = 2;
        shape = {vector.size(), width};
        strides = {sizeof(Scalar) * width, sizeof(Scalar)};
    }

    py::array_t<Scalar> result(py::buffer_info(
    (void *) (&vector[0]), static_cast<py::ssize_t>(sizeof(Scalar)),
            py::format_descriptor<Scalar>::format(),
            ndim,
            shape,
            strides
    ));

    return result;

}

PYBIND11_MODULE(pyct_icp, m) {
    /// LiDARFrame : A wrapper around a vector of ct_icp::Point3D
    PYBIND11_NUMPY_DTYPE(PyLiDARPoint, raw_point, pt, alpha_timestamp, timestamp, frame_index);

    py::class_<LiDARFrame, std::shared_ptr<LiDARFrame>>(m, "LiDARFrame")
            .def(py::init())
            .def("SetFrame", &LiDARFrame::SetFrame)
            .def("GetStructuredArrayCopy", [](LiDARFrame &self) {
                return self.GetWrappingArray();
            })
            .def("GetStructuredArrayRef", [](py::object &self) {
                auto &self_frame = py::cast<LiDARFrame &>(self);
                return self_frame.GetWrappingArray(self);
            });

    py::class_<ct_icp::TrajectoryFrame, std::shared_ptr<ct_icp::TrajectoryFrame>>(m, "TrajectoryFrame")
            .def(py::init())
                    STRUCT_READWRITE(ct_icp::TrajectoryFrame, begin_R)
                    STRUCT_READWRITE(ct_icp::TrajectoryFrame, end_R)
                    STRUCT_READWRITE(ct_icp::TrajectoryFrame, end_t)
                    STRUCT_READWRITE(ct_icp::TrajectoryFrame, begin_t)
            .def("MidPose", &ct_icp::TrajectoryFrame::MidPose);


    /// ODOMETRY
    py::enum_<ct_icp::LEAST_SQUARES>(m, "LEAST_SQUARES")
            ADD_VALUE(ct_icp::LEAST_SQUARES, CAUCHY)
            ADD_VALUE(ct_icp::LEAST_SQUARES, TOLERANT)
            ADD_VALUE(ct_icp::LEAST_SQUARES, TRUNCATED)
            ADD_VALUE(ct_icp::LEAST_SQUARES, HUBER)
            ADD_VALUE(ct_icp::LEAST_SQUARES, STANDARD)
            .export_values();

    py::enum_<ct_icp::INITIALIZATION>(m, "INITIALIZATION")
            ADD_VALUE(ct_icp::INITIALIZATION, INIT_NONE)
            ADD_VALUE(ct_icp::INITIALIZATION, INIT_CONSTANT_VELOCITY)
            .export_values();

    py::enum_<ct_icp::ICP_DISTANCE>(m, "ICP_DISTANCE")
            ADD_VALUE(ct_icp::ICP_DISTANCE, POINT_TO_PLANE)
            ADD_VALUE(ct_icp::ICP_DISTANCE, CT_POINT_TO_PLANE)
            .export_values();

    py::enum_<ct_icp::MOTION_COMPENSATION>(m, "MOTION_COMPENSATION")
            ADD_VALUE(ct_icp::MOTION_COMPENSATION, NONE)
            ADD_VALUE(ct_icp::MOTION_COMPENSATION, CONSTANT_VELOCITY)
            ADD_VALUE(ct_icp::MOTION_COMPENSATION, ITERATIVE)
            ADD_VALUE(ct_icp::MOTION_COMPENSATION, CONTINUOUS)
            .export_values();

    py::enum_<ct_icp::CT_ICP_SOLVER>(m, "CT_ICP_SOLVER")
            ADD_VALUE(ct_icp::CT_ICP_SOLVER, CERES)
            ADD_VALUE(ct_icp::CT_ICP_SOLVER, GN)
            .export_values();

    py::class_<ct_icp::SequenceInfo>(m, "SequenceInfo")
            .def(py::init())
                    STRUCT_READWRITE(ct_icp::SequenceInfo, sequence_size)
                    STRUCT_READWRITE(ct_icp::SequenceInfo, sequence_name)
                    STRUCT_READWRITE(ct_icp::SequenceInfo, sequence_id);


    py::class_<ct_icp::CTICPOptions,
            std::shared_ptr<ct_icp::CTICPOptions>>(m, "CTICPOptions")
            .def(py::init())
                    STRUCT_READWRITE(ct_icp::CTICPOptions, distance)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, num_iters_icp)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, min_number_neighbors)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, voxel_neighborhood)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, max_number_neighbors)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, max_dist_to_plane_ct_icp)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, norm_x_end_iteration_ct_icp)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, debug_print)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, point_to_plane_with_distortion)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, distance)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, num_closest_neighbors)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, beta_location_consistency)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, beta_constant_velocity)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, loss_function)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, ls_max_num_iters)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, ls_num_threads)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, size_voxel_map)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, ls_sigma)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, solver)
                    STRUCT_READWRITE(ct_icp::CTICPOptions, ls_tolerant_min_threshold);

    py::class_<ct_icp::OdometryOptions>(m, "OdometryOptions")
            .def(py::init())
                    STRUCT_READWRITE(ct_icp::OdometryOptions, voxel_size)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, sample_voxel_size)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, max_distance)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, max_num_points_in_voxel)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, motion_compensation)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, debug_print)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, min_distance_points)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, initialization)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, distance_error_threshold)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, robust_registration)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, robust_fail_early)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, robust_full_voxel_threshold)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, robust_num_attempts)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, robust_max_voxel_neighborhood)
                    STRUCT_READWRITE(ct_icp::OdometryOptions, ct_icp_options);


    m.def("DefaultDrivingProfile", &ct_icp::OdometryOptions::DefaultDrivingProfile);
    m.def("DefaultSlowOutdoorProfile", &ct_icp::OdometryOptions::DefaultSlowOutdoorProfile);

    using RegSummary = ct_icp::Odometry::RegistrationSummary;
    py::class_<PyRegistrationSummary,
            std::shared_ptr<PyRegistrationSummary>>(m, "RegistrationSummary")
            .def_readonly("sample_size", &RegSummary::sample_size)
            .def_readonly("number_keypoints", &RegSummary::number_keypoints)
            .def_readonly("distance_correction", &RegSummary::distance_correction)
            .def_readonly("relative_distance", &RegSummary::relative_distance)
            .def_readonly("success", &RegSummary::success)
            .def_readonly("frame", &RegSummary::frame)
            .def_readonly("frame", &RegSummary::error_message)
            .def_readonly("frame", &RegSummary::number_of_attempts)
            .def_readonly("points", &PyRegistrationSummary::lidar_points);


    py::class_<ct_icp::Odometry,
            std::shared_ptr<ct_icp::Odometry>>(m, "Odometry")
            .def(py::init([](ct_icp::OdometryOptions &options) {
                return ct_icp::Odometry(options);
            }))
            .def("RegisterFrame", [](ct_icp::Odometry &odometry, const LiDARFrame &frame) {
                return PyRegistrationSummary(odometry.RegisterFrame(frame.points));
            })
            .def("MapSize", &ct_icp::Odometry::MapSize)
            .def("Trajectory", &ct_icp::Odometry::Trajectory)
            .def("GetLocalMap", [](const ct_icp::Odometry &self) {
                // Convert to numpy
                return vector_to_ndarray<Eigen::Vector3d, double,
                        Eigen::aligned_allocator<Eigen::Vector3d>>(self.GetLocalMap());
            });


    /// DATASETS
    py::enum_<ct_icp::DATASET>(m, "CT_ICP_DATASET")
            ADD_VALUE(ct_icp::DATASET, KITTI_raw)
            ADD_VALUE(ct_icp::DATASET, KITTI_CARLA)
            ADD_VALUE(ct_icp::DATASET, NCLT)
            ADD_VALUE(ct_icp::DATASET, PLY_DIRECTORY)
            ADD_VALUE(ct_icp::DATASET, KITTI_360)
            .export_values();

    py::class_<ct_icp::DatasetOptions>(m, "DatasetOptions")
            .def(py::init())
                    STRUCT_READWRITE(ct_icp::DatasetOptions, dataset)
                    STRUCT_READWRITE(ct_icp::DatasetOptions, root_path)
                    STRUCT_READWRITE(ct_icp::DatasetOptions, fail_if_incomplete)
                    STRUCT_READWRITE(ct_icp::DatasetOptions, min_dist_lidar_center)
                    STRUCT_READWRITE(ct_icp::DatasetOptions, max_dist_lidar_center)
                    STRUCT_READWRITE(ct_icp::DatasetOptions, nclt_num_aggregated_pc);

    py::class_<ct_icp::DatasetSequence, std::shared_ptr<ct_icp::DatasetSequence>>(m, "DatasetSequence")
            .def("HasNext", &ct_icp::DatasetSequence::HasNext)
            .def("Next", [](ct_icp::DatasetSequence &iterator) {
                LiDARFrame frame;
                frame.points = iterator.Next();
                return frame;
            })
            .def("NumFrames", &ct_icp::DatasetSequence::NumFrames)
            .def("WithRandomAccess", &ct_icp::DatasetSequence::WithRandomAccess)
            .def("Frame", [](ct_icp::DatasetSequence &self, int index_frame) {
                CHECK(self.WithRandomAccess()) << "Random Access is not available for the dataset";
                LiDARFrame frame;
                frame.points = self.Frame(size_t(index_frame));
                return frame;
            });

    m.def("sequence_name", &ct_icp::sequence_name);
    m.def("get_sequences", &ct_icp::get_sequences);
    m.def("has_ground_truth", &ct_icp::has_ground_truth);
    m.def("get_dataset_sequence", &ct_icp::get_dataset_sequence);
    m.def("load_sensor_ground_truth", &ct_icp::load_sensor_ground_truth);
    m.def("load_ground_truth", &ct_icp::load_sensor_ground_truth);

}