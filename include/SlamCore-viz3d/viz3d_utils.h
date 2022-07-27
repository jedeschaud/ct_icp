#ifndef SlamCore_VIZ3D_UTILS_H
#define SlamCore_VIZ3D_UTILS_H

#ifdef SLAM_WITH_VIZ3D

#include <vtkSmartPointer.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>
#include <vtkActor.h>
#include <vtkLookupTable.h>

#include <SlamCore/types.h>
#include "SlamCore/pointcloud.h"
#include "SlamCore/reactors/reactor.h"

namespace slam {

    /*!
     *  Converts a slam::SE3 to a pointer to a vtkTransform
     */
    inline vtkSmartPointer<vtkTransform> slam_to_vtk_transform(const slam::SE3 &pose) {
        vtkNew<vtkTransform> vtk_transform;
        Eigen::Matrix4d mat = pose.Matrix().transpose();
        vtk_transform->Translate(pose.tr.data());
        vtk_transform->SetMatrix(mat.data());
        return vtk_transform;
    }

    /*!
     * Creates a PolyData from a Point Cloud
     *
     * @param world_points  Whether to use the world points or the raw points for the PolyData
     */
    vtkSmartPointer<vtkPolyData> polydata_from_points(const std::vector<slam::WPoint3D> &points,
                                                      bool world_points = true);


    template<typename IteratorT, typename ScalarT>
    vtkSmartPointer<vtkPolyData> _polydata_from_points(IteratorT begin, IteratorT end);

    /*!
     * Creates a PolyData from an iterator of points
     *
     * @tparam  IteratorT an iterator of Eigen::Vector3d
     */
    template<typename IteratorT>
    std::enable_if_t<std::is_same_v<typename IteratorT::value_type, Eigen::Vector3d>,
            vtkSmartPointer<vtkPolyData>> polydata_from_points(IteratorT begin, IteratorT end) {
        return _polydata_from_points<IteratorT, double>(begin, end);
    };

    template<typename IteratorT>
    std::enable_if_t<std::is_same_v<typename IteratorT::value_type, Eigen::Vector3f>,
            vtkSmartPointer<vtkPolyData>> polydata_from_points(IteratorT begin, IteratorT end) {
        return _polydata_from_points<IteratorT, float>(begin, end);
    };

    /*!
     * Creates a PolyData from an iterator of points
     *
     * @param element_property_pairs The vector of pairs {element_name, property_name} defining
     *                               all the scalar fields to add to the Poly data
     */
    vtkSmartPointer<vtkPolyData> polydata_from_pointcloud(const slam::PointCloud &pc,
                                                          std::vector<std::pair<std::string,
                                                                  std::string>> element_property_pairs);

    /*!
     * Creates a PolyData from an iterator of points
     *
     * Adds all properties of the point cloud to the PolyData
     */
    vtkSmartPointer<vtkPolyData> polydata_from_pointcloud(const slam::PointCloud &pc);


    /*!
     *  Creates a VTK Actor from an iterator of points
     */
    vtkSmartPointer<vtkActor> vtk_actor_from_pointcloud(const slam::PointCloud &pc);

    /*!
     * Creates a PolyData from a set of Poses, where for each pose, 3 lines are defined
     *
     * @tparam IteratorT An iterator of slam::SE3 poses
     * @param  Scale    The scale of the coordinate frames
     */
    template<typename IteratorT>
    vtkSmartPointer<vtkPolyData> polydata_from_poses(IteratorT begin, IteratorT end, double scale = 1.0);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS                                                                                              ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename IteratorT>
    vtkSmartPointer<vtkPolyData> polydata_from_poses(IteratorT begin, IteratorT end, double scale) {
        static_assert(std::is_same_v<typename IteratorT::value_type, slam::SE3>,
                      "The iterator is not an iterator of poses");
        const auto num_poses = std::distance(begin, end);
        const auto num_points = num_poses * 4;
        const auto num_lines = num_poses * 3;

        // Setup the array of points
        auto vtk_points = vtkSmartPointer<vtkPoints>::New();
        vtk_points->SetDataTypeToFloat();
        vtk_points->Allocate(num_points);
        vtk_points->SetNumberOfPoints(num_points);
        vtk_points->GetData()->SetName("Points_XYZ");

        vtkNew<vtkPolyData> poly_data;
        poly_data->SetPoints(vtk_points.GetPointer());

        // Assign for each cell vertex indices
        auto cell_ids = vtkSmartPointer<vtkIdTypeArray>::New();
        cell_ids->SetNumberOfValues(num_lines * 3);
        vtkIdType *ids = cell_ids->GetPointer(0);
        for (vtkIdType pose_id = 0; pose_id < num_poses; ++pose_id) {
            vtkIdType origin_pid = 4 * pose_id;
            for (auto line_id(0); line_id < 3; line_id++) {
                auto cell_id = pose_id * 9 + line_id * 3;
                ids[cell_id] = 2; // num points in the cell = 2
                ids[cell_id + 1] = origin_pid; // origin pid
                ids[cell_id + 2] = origin_pid + 1 + line_id; // pid of the endpoint of the line
            }
        }

        vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
        cellArray->SetCells(num_lines, cell_ids.GetPointer());
        poly_data->SetLines(cellArray);

        auto color_field = vtkSmartPointer<vtkFloatArray>::New();
        color_field->Allocate(num_lines);
        color_field->SetName("Coordinates");
        color_field->SetNumberOfTuples(num_points);
        poly_data->GetCellData()->AddArray(color_field);
        auto pose_idx = 0;
        auto current = begin;
        while (current < end) {
            slam::SE3 pose = *current;
            Eigen::Matrix3d rotation = pose.Rotation();

            Eigen::Vector3f e0 = pose.tr.cast<float>();
            Eigen::Vector3f x = e0 + (rotation * Eigen::Vector3d::UnitX() * scale).cast<float>();
            Eigen::Vector3f y = e0 + (rotation * Eigen::Vector3d::UnitY() * scale).cast<float>();
            Eigen::Vector3f z = e0 + (rotation * Eigen::Vector3d::UnitZ() * scale).cast<float>();

            // Set Points value
            auto base_point_idx = 4 * pose_idx;
            vtk_points->SetPoint(base_point_idx, e0.data());
            vtk_points->SetPoint(base_point_idx + 1, x.data());
            vtk_points->SetPoint(base_point_idx + 2, y.data());
            vtk_points->SetPoint(base_point_idx + 3, z.data());

            // Set the Cell Data (ie the color_field)
            auto base_cell_idx = 3 * pose_idx;

            for (auto line_id(0); line_id < 3; line_id++) {
                color_field->SetValue(base_cell_idx, 0.f); // X coordinate
                color_field->SetValue(base_cell_idx + 1, 0.5f); // Y coordinate
                color_field->SetValue(base_cell_idx + 2, 1.0f); // Z coordinate
            }

            pose_idx++;
            current++;
        }
        poly_data->GetCellData()->SetActiveScalars("Coordinates");

        return poly_data;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename IteratorT, typename ScalarT>
    vtkSmartPointer<vtkPolyData> _polydata_from_points(IteratorT begin, IteratorT end) {
        typedef Eigen::Matrix<ScalarT, 3, 1> PointVecT;
        static_assert(std::is_same_v<typename IteratorT::value_type, PointVecT>,
                      "The iterator is not an iterator of Eigen::Vector3d");
        const auto num_poses = std::distance(begin, end);
        const auto num_points = num_poses;
        auto poly_data = vtkSmartPointer<vtkPolyData>::New();

        // Setup the array of points
        auto vtk_points = vtkSmartPointer<vtkPoints>::New();
        vtk_points->SetDataTypeToFloat();
        vtk_points->Allocate(num_points);
        vtk_points->SetNumberOfPoints(num_points);
        vtk_points->GetData()->SetName("Points_XYZ");
        poly_data->SetPoints(vtk_points.GetPointer());

        // Assign for each cell vertex indices
        auto cell_ids = vtkSmartPointer<vtkIdTypeArray>::New();
        cell_ids->SetNumberOfValues(num_points * 2);
        vtkIdType *ids = cell_ids->GetPointer(0);
        for (vtkIdType pid = 0; pid < num_poses; ++pid) {
            ids[pid * 2] = 1;
            ids[pid * 2 + 1] = pid;
        }

        vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
        cellArray->SetCells(num_points, cell_ids.GetPointer());
        poly_data->SetVerts(cellArray);

        {
            // Double Field for timestamps
            std::map<std::string, vtkSmartPointer<vtkFloatArray>> fields{
                    {"X", vtkSmartPointer<vtkFloatArray>::New()},
                    {"Y", vtkSmartPointer<vtkFloatArray>::New()},
                    {"Z", vtkSmartPointer<vtkFloatArray>::New()},
            };
            for (auto &pair: fields) {
                pair.second->Allocate(num_points);
                pair.second->SetNumberOfTuples(num_points);
                pair.second->SetName(pair.first.c_str());
                poly_data->GetPointData()->AddArray(pair.second);
            }

            auto current = begin;
            auto idx(0);
            while (current < end) {
                PointVecT xyzT = (*current);
                Eigen::Vector3f xyz = xyzT.template cast<float>();
                vtk_points->SetPoint(idx, xyz.data());
                fields["X"].GetPointer()->SetValue(idx, (float) xyz.x());
                fields["Y"].GetPointer()->SetValue(idx, (float) xyz.y());
                fields["Z"].GetPointer()->SetValue(idx, (float) xyz.z());
                current++;
                idx++;
            }
        }
        poly_data->GetPointData()->SetActiveScalars("Z");
        return poly_data;
    }


    /* -------------------------------------------------------------------------------------------------------------- */


}

#endif // SLAM_WITH_VIZ3D
#endif //SlamCore_VIZ3D_UTILS_H
