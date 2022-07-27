#include <gtest/gtest.h>

#include <thread>
#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <SlamCore-viz3d/viz3d_utils.h>
#include <SlamCore-viz3d/viz3d_windows.h>
#include <SlamCore/experimental/iterator/transform_iterator.h>

#include <viz3d/ui.h>
#include <viz3d/vtk_window.h>

#include <vtkPolyDataMapper.h>

// --------------------------
TEST(viz3d_utils, pointcloud_to_polydata) {

    auto pc = slam::PointCloud::DefaultXYZPtr<double>();
    pc->resize(1000);
    pc->AddDefaultWorldPointsField();
    pc->AddDefaultNormalsField();
    pc->AddDefaultTimestampsField();
    pc->AddDefaultIntensityField();
    pc->SetWorldPointsField(slam::PointCloud::Field{pc->GetXYZField()});
    auto polydata = slam::polydata_from_pointcloud(*pc);

    auto num_arrays = polydata->GetPointData()->GetNumberOfArrays();
    for (auto i(0); i < num_arrays; ++i) {
        auto array = polydata->GetPointData()->GetArray(i);
        std::cout << array->GetName();
    }
}

// --------------------------
TEST(viz3d_utils, poses_to_polydata) {

    {
        auto poses = std::make_shared<std::vector<slam::Pose>>(1000);
        for (auto &pose: *poses) {
            pose.pose.tr = Eigen::Vector3d(0., 42., 1.0);
        }

        const auto &_poses = *poses;
        auto [begin, end] = slam::make_transform_collection(_poses, slam::PoseConversion());
        auto polydata = slam::polydata_from_poses(begin, end);

        {
            const auto [_begin, _end] = slam::make_transform_collection(*poses, slam::PoseConversion());
            auto _current = _begin;
            while (_current < _end) {
                slam::SE3 pose = *_current;

                ASSERT_EQ(pose.tr.y(), 42);
                ASSERT_EQ(pose.tr.x(), 0.);
                ASSERT_EQ(pose.tr.z(), 1.);
                _current++;
            }
        }
    }
}

// --------------------------
TEST(viz3d_utils, test) {

    const auto kNumPoints = 10000;
    std::vector<slam::WPoint3D> points, points_bis;
    points.reserve(kNumPoints);
    points_bis.reserve(kNumPoints);

    for (auto k(0); k < kNumPoints; ++k) {
        slam::WPoint3D new_point;
        new_point.RawPoint() = Eigen::Vector3d::Random();
        new_point.RawPoint().z() = 0.;
        new_point.Timestamp() = std::abs(new_point.RawPoint().x());
        points.push_back(new_point);

        new_point.RawPoint() = Eigen::Vector3d::Random();
        new_point.RawPoint().x() = 0.;
        new_point.Timestamp() = std::pow(new_point.RawPoint().y(), 2) + new_point.RawPoint().z();
        points_bis.push_back(new_point);
    }

    auto poly_data = slam::polydata_from_points(points, false);
    auto poly_data_bis = slam::polydata_from_points(points_bis, false);

    std::vector<slam::SE3> poses(100);
    for (auto &pose: poses) {
        pose.quat = Eigen::Quaterniond::UnitRandom();
        pose.tr = Eigen::Vector3d::Random();
    }

    for (auto &point: points)
        point.RawPoint().z() += 10.;
    auto pc = slam::PointCloud::WrapVector(points, slam::WPoint3D::DefaultSchema(), "raw_point");
    auto poly_data_2 = slam::polydata_from_pointcloud(pc);

    // Add Colors to Poly Data:
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");
    for (int i(0); i < points.size(); i++) {
        Eigen::Vector3f random = Eigen::Vector3f::Random().cwiseAbs() * 255.f;
        unsigned char color[3];
        if (i % 2 == 0) {
            color[0] = 0;
            color[1] = 255;
            color[2] = 0;
        } else {
            color[0] = 200;
            color[1] = 33;
            color[2] = 0;
        }
        colors->InsertNextTypedTuple(color);
    }
    poly_data_2->GetPointData()->SetScalars(colors);
    poly_data_2->GetPointData()->SetActiveScalars("Colors");

    auto poly_data_poses = slam::polydata_from_poses(poses.begin(), poses.end(), 0.1);

    auto &gui = viz3d::GUI::Instance();
    auto window = std::make_shared<slam::MultiPolyDataWindow>("VTK Window");
//    window->InitializeVTKContext();
    window->AddPolyData("Points", 0, poly_data);
    window->AddPolyData("Points3", 1, poly_data_2);
    window->AddPolyData("Points_Bis", 0, poly_data_bis);
    window->AddPolyData("Poses", 0, poly_data_poses);
    window->InitializeWindow();

    std::thread gui_thread{viz3d::GUI::LaunchMainLoop, "GUI"};

#ifndef VIZ3D_WAIT_FOR_CLOSE
    std::this_thread::sleep_for(std::chrono::duration<double, std::ratio<1, 1>>(2.0));
#endif
    gui.SignalClose();
    gui_thread.join();

    ASSERT_TRUE(true);
}

// --------------------------
TEST(viz3d_utils, conversion) {

    auto &gui = viz3d::GUI::Instance();

    auto window = std::make_shared<slam::PointCloudQueueVTKWindow>("Window");
    gui.AddWindow(window);
    std::thread gui_thread{viz3d::GUI::LaunchMainLoop, "GUI"};

    int num_iters = 10;
#ifndef VIZ3D_WAIT_FOR_CLOSE
    num_iters = 10;
#endif
    for (int i(0); i < num_iters; i++) {
        std::this_thread::sleep_for(std::chrono::duration<double, std::ratio<1, 1>>(0.5));

        auto default_pc = slam::PointCloud::DefaultXYZPtr<float>();
        default_pc->resize(1000);
        auto view = default_pc->XYZ<float>();
        auto begin = view.begin();
        auto end = view.end();
        while (begin < end) {
            Eigen::Vector3f random = Eigen::Vector3f::Random();
            random.z() = 0.;
            *begin = random;
            begin++;
        }

        window->PushNewFrame(default_pc);
    }

    gui.SignalClose();
    gui.ClearWindows();
    gui_thread.join();
}



