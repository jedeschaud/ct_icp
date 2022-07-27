#ifdef SLAM_WITH_VIZ3D

#include "SlamCore-viz3d/viz3d_utils.h"

#include <colormap/colormap.hpp>
#include <SlamCore/generic_tools.h>
#include <SlamCore/experimental/iterator/transform_iterator.h>

#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkTypeUInt32Array.h>
#include <vtkTypeUInt64Array.h>
#include <vtkTypeInt16Array.h>
#include <vtkTypeUInt16Array.h>
#include <vtkTypeUInt8Array.h>
#include <vtkTypeInt8Array.h>
#include <vtkPolyDataMapper.h>

namespace slam {

//    /* -------------------------------------------------------------------------------------------------------------- */
//    viz::ArrayV3f slam_to_viz3d_pc(const std::vector<slam::WPoint3D> &points, bool world_points) {
//        viz::ArrayV3f new_points(points.size());
//        std::transform(points.begin(), points.end(), new_points.begin(), [world_points](const auto &wpoint) {
//            return (world_points ? wpoint.world_point : wpoint.raw_point.point).template cast<float>();
//        });
//        return new_points;
//    }
//
//    namespace {
//
//        std::string GetColorMap(COLOR_SCHEME scheme) {
//            switch (scheme) {
//                case COLOR_SCHEME::MAGMA:
//                    return "magma";
//                case COLOR_SCHEME::JET:
//                    return "jet";
//                case COLOR_SCHEME::VIRIDIS:
//                    return "viridis";
//                default:
//                    LOG(INFO) << "The color scheme " << scheme << " does not exist" << std::endl;
//                    return "jet";
//            }
//        }
//    }
//
//    /* -------------------------------------------------------------------------------------------------------------- */
//    viz::ArrayV3f get_viz3d_color(const std::vector<double> &scalars, bool normalize, COLOR_SCHEME cmap) {
//        std::vector<double> copy;
//        viz::ArrayV3f viz_points(scalars.size());
//        if (normalize) {
//            copy = scalars;
//            double min_value = *std::min_element(scalars.begin(), scalars.end());
//            double max_value = *std::max_element(scalars.begin(), scalars.end());
//            double scalar;
//            for (auto &value: copy) {
//                scalar = (value - min_value) / (max_value - min_value);
//                value = scalar;
//            }
//        }
//
//        auto palette = colormap::palettes.at(GetColorMap(cmap)).rescale(0, 1);
//        for (auto i(0); i < scalars.size(); ++i) {
//            double s = normalize ? copy[i] : std::min(std::max(scalars[i], 0.), 1.);
//            colormap::rgb value = palette(s);
//            std::uint8_t *rgb_color_ptr = reinterpret_cast<std::uint8_t *>(&value);
//            Eigen::Vector3f rgb((float) rgb_color_ptr[0] / 255.0f,
//                                (float) rgb_color_ptr[1] / 255.0f, (float) rgb_color_ptr[2] / 255.0f);
//            viz_points[i] = rgb;
//        }
//
//        return viz_points;
//    }
//
//    /* -------------------------------------------------------------------------------------------------------------- */
//    viz::ArrayV3f get_field_color(const std::vector<slam::WPoint3D> &points, COLOR_SCHEME cmap, COLOR_FIELD cfield) {
//        std::vector<double> scalars(points.size());
//        double scalar;
//        for (auto i(0); i < points.size(); ++i) {
//            auto &point = points[i];
//            switch (cfield) {
//                case X:
//                    scalar = point.WorldPointConst().x();
//                    break;
//                case Y:
//                    scalar = point.WorldPointConst().y();
//                    break;
//                case Z:
//                    scalar = point.WorldPointConst().z();
//                    break;
//                case T:
//                default:
//                    scalar = point.TimestampConst();
//                    break;
//            }
//            scalars[i] = scalar;
//        }
//
//        return get_viz3d_color(scalars, true, cmap);
//    }
//
//    /* -------------------------------------------------------------------------------------------------------------- */
//    viz::ArrayM4f slam_to_viz3d_poses(const std::vector<slam::Pose> &poses) {
//        return transform_vector<viz::glMatrix4f,
//                viz::ArrayM4f::allocator_type>(poses,
//                                               [](const auto &pose) {
//                                                   Eigen::Matrix4f mat = pose.Matrix().template cast<float>();
//                                                   return mat;
//                                               });;
//    }
//
//    /* -------------------------------------------------------------------------------------------------------------- */
//    Eigen::Vector3f scalar_to_color(double value, COLOR_SCHEME cmap) {
//        CHECK(0. <= value && value <= 1.) << "The value must be in the range [0, 1.]" << std::endl;
//        const auto &palette = colormap::palettes.at(GetColorMap(cmap));
//        auto rgb = palette(value);
//        auto &_rgb = *reinterpret_cast<const std::array<unsigned char, 3> *> (&rgb);
//        return Eigen::Vector3f(float(_rgb[0]) / 255.f, float(_rgb[1]) / 255.f, float(_rgb[2]) / 255.f);
//    }


    /* -------------------------------------------------------------------------------------------------------------- */
    vtkSmartPointer<vtkPolyData> polydata_from_points(const std::vector<slam::WPoint3D> &points, bool world_points) {
        vtkSmartPointer<vtkPolyData> poly_data;
        if (world_points) {
            auto begin = slam::make_transform(points.begin(), WorldPointConversion());
            auto end = slam::make_transform(points.end(), WorldPointConversion());
            poly_data = polydata_from_points(begin, end);
        } else {
            auto begin = slam::make_transform(points.begin(), RawPointConversion());
            auto end = slam::make_transform(points.end(), RawPointConversion());
            poly_data = polydata_from_points(begin, end);
        }

        // Add the Timestamps
        auto num_points = points.size();
        vtkNew<vtkFloatArray> pair;
        pair->Allocate(num_points);
        pair->SetNumberOfTuples(num_points);
        pair->SetName("T");
        poly_data->GetPointData()->AddArray(pair);

        for (auto pidx(0); pidx < points.size(); pidx++) {
            pair.GetPointer()->SetValue(pidx, float(points[pidx].TimestampConst()));
        }

        return poly_data;
    }

    namespace {
        vtkSmartPointer<vtkDataArray> GetVTKDataArrayFromSlamType(slam::PROPERTY_TYPE type) {
            switch (type) {
                case slam::FLOAT64:
                    return vtkSmartPointer<vtkDoubleArray>::New();
                case slam::FLOAT32:
                    return vtkSmartPointer<vtkFloatArray>::New();
                case slam::INT64:
                    return vtkSmartPointer<vtkTypeInt64Array>::New();
                case slam::UINT64:
                    return vtkSmartPointer<vtkTypeUInt64Array>::New();
                case slam::INT32:
                    return vtkSmartPointer<vtkTypeInt32Array>::New();
                case slam::UINT32:
                    return vtkSmartPointer<vtkTypeUInt32Array>::New();
                case slam::INT16:
                    return vtkSmartPointer<vtkTypeInt16Array>::New();
                case slam::UINT16:
                    return vtkSmartPointer<vtkTypeUInt16Array>::New();
                case slam::INT8:
                    return vtkSmartPointer<vtkTypeInt8Array>::New();
                case slam::UINT8:
                    return vtkSmartPointer<vtkTypeUInt8Array>::New();
                default:
                    throw std::runtime_error("Not implemented exception");
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    vtkSmartPointer<vtkPolyData> polydata_from_pointcloud(const PointCloud &pc) {
        std::vector<std::pair<std::string, std::string>> pairs;
        auto &collection = pc.GetCollection();

        std::optional<slam::PointCloud::Field> normals_field, rgb_field;
        if (pc.HasNormals())
            normals_field = pc.GetNormalsField();
        if (pc.HasRGB())
            rgb_field = pc.GetRGBField();

        for (auto item_idx(0); item_idx < collection.NumItemsInSchema(); item_idx++) {
            for (auto &elem_name: collection.GetItemInfo(item_idx).item_schema.GetElementNames()) {

                // Skip over RGB element
                if (rgb_field && (rgb_field->element_name) && elem_name == *rgb_field->element_name)
                    continue;

                // Skip over Normal element
                if (normals_field && (normals_field->element_name) && elem_name == *normals_field->element_name)
                    continue;

                auto &elem_info = collection.GetElement(elem_name);
                for (auto &pty_info: elem_info.properties)
                    pairs.push_back({elem_name, pty_info.property_name});
            }
        }
        auto poly_data = polydata_from_pointcloud(pc, pairs);

        auto set_vec3_field = [&](const ProxyView<Eigen::Vector3f> &proxy, const std::string &field_name,
                                  bool normalize = false) {
            vtkNew<vtkUnsignedCharArray> normals_colors;
            normals_colors->SetNumberOfComponents(3);
            normals_colors->SetName(field_name.c_str());

            Eigen::Vector3f data;
            for (auto normal_proxy: proxy) {
                data = normal_proxy;
                data = data.cwiseAbs().normalized();
                unsigned char rgb[3];
                rgb[0] = (unsigned char) (data.x() * 255.f);
                rgb[1] = (unsigned char) (data.y() * 255.f);
                rgb[2] = (unsigned char) (data.z() * 255.f);
                normals_colors->InsertNextTypedTuple(rgb);
            }
            poly_data->GetPointData()->SetScalars(normals_colors);
        };
        if (normals_field)
            set_vec3_field(pc.NormalsProxy<Eigen::Vector3f>(), "Normals", true);
        if (rgb_field)
            set_vec3_field(pc.RGBProxy<Eigen::Vector3f>(), "RGB", false);

        return poly_data;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    vtkSmartPointer<vtkPolyData>
    polydata_from_pointcloud(const PointCloud &pc,
                             std::vector<std::pair<std::string, std::string>> element_property_pairs) {
        auto points = pc.XYZConst<float>();
        auto poly_data = slam::polydata_from_points(points.begin(), points.end());

        // Add all properties as scalars
        auto &collection = pc.GetCollection();
        for (auto &pair: element_property_pairs) {
            auto &elem_info = collection.GetElement(pair.first);
            auto &pty_info = elem_info.GetProperty(pair.second);

            for (auto dim_id(0); dim_id < pty_info.dimension; dim_id++) {

                std::string ss_name =
                        "item_" + std::to_string(0) + "_" + elem_info.element_name + "_" + pty_info.property_name;
                if (pty_info.dimension > 1)
                    ss_name = ss_name + "_" + std::to_string(dim_id);

                vtkSmartPointer<vtkDataArray> array = GetVTKDataArrayFromSlamType(pty_info.type);
                array->SetNumberOfComponents(1);
                array->SetNumberOfTuples(pc.size());

                array->SetName(ss_name.c_str());

                // Add the array to the poly data
                poly_data->GetPointData()->AddArray(array);

                // Copies the data into each property
                auto properties = pc.GetCollection().property_proxy<double>(elem_info.element_name,
                                                                            pty_info.property_name,
                                                                            dim_id);
                auto idx(0);
                auto current = properties.begin();
                while (current != properties.end()) {
                    double current_value = *current;
                    array->SetComponent(idx, 0, current_value);
                    idx++;
                    current++;
                }
            }
        }
        return poly_data;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    vtkSmartPointer<vtkActor> vtk_actor_from_pointcloud(const PointCloud &pc) {
        auto polydata = polydata_from_pointcloud(pc);
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);
        vtkSmartPointer<vtkActor> new_actor = vtkSmartPointer<vtkActor>::New();
        new_actor->SetMapper(mapper);
        return new_actor;
    }

} // namespace

#endif
