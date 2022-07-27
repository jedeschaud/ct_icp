#include <thread>

#include <gtest/gtest.h>

#include <SlamCore/experimental/synthetic.h>

#ifdef SLAM_WITH_VIZ3D

#include <SlamCore-viz3d/viz3d_utils.h>
#include <SlamCore-viz3d/viz3d_windows.h>
#include <viz3d/ui.h>
#include <vtkPointData.h>

#endif

TEST(Synthetic, io) {
    slam::Triangle triangle, triangle_bis;
    triangle.Points()[0] = Eigen::Vector3d::Random();
    triangle.Points()[1] = Eigen::Vector3d::Random();
    triangle.Points()[2] = Eigen::Vector3d::Random();

    YAML::Node node = triangle.ToYAMLNode();
    triangle_bis.FromYAMLNode(node);
    for (int i(0); i < 3; ++i)
        ASSERT_EQ((triangle.Points()[i] - triangle_bis.Points()[i]).norm(), 0.);

    Eigen::Vector3d point = triangle.Barycenter();
    double distance = triangle.Distance(point);

    ASSERT_LE(distance, 1.e-16);
}


TEST(Synthetic, GeneratePoints) {
#ifdef SLAM_WITH_VIZ3D

    std::string scene_yaml = R"(
triangles:
  # GROUND
  - [ [ 0, 0, 0 ], [ 0, 10, 0 ], [ 5, 10, 0 ] ]
  - [ [ 0, 0, 0 ], [ 5, 0, 0 ], [ 5, 10, 0 ] ]

  - [ [ 5, 0, 0 ], [ 5, 10, 0 ], [ 10, 10, 0 ] ]
  - [ [ 5, 0, 0 ], [ 10, 0, 0 ], [ 10, 10, 0 ] ]
  - [ [ 5, 10, 0 ], [ 5, 20, 0 ], [ 10, 20, 0 ] ]
  - [ [ 5, 10, 0 ], [ 10, 10, 0 ], [ 10, 20, 0 ] ]

  - [ [ 10, 0, 0 ], [ 10, 10, 0 ], [ 15, 10, 0 ] ]
  - [ [ 10, 0, 0 ], [ 15, 0, 0 ], [ 15, 10, 0 ] ]
  - [ [ 10, 10, 0 ], [ 10, 20, 0 ], [ 15, 20, 0 ] ]
  - [ [ 10, 10, 0 ], [ 15, 10, 0 ], [ 15, 20, 0 ] ]

  - [ [ 15, 0, 0 ], [ 15, 10, 0 ], [ 20, 10, 0 ] ]
  - [ [ 15, 0, 0 ], [ 20, 0, 0 ], [ 20, 10, 0 ] ]
  - [ [ 15, 10, 0 ], [ 15, 20, 0 ], [ 20, 20, 0 ] ]
  - [ [ 15, 10, 0 ], [ 20, 10, 0 ], [ 20, 20, 0 ] ]

  - [ [ 0, 20, 0 ], [ 0, 30, 0 ], [ 5, 30, 0 ] ]
  - [ [ 0, 20, 0 ], [ 5, 20, 0 ], [ 5, 30, 0 ] ]

  - [ [ 5, 20, 0 ], [ 5, 30, 0 ], [ 10, 30, 0 ] ]
  - [ [ 5, 20, 0 ], [ 10, 20, 0 ], [ 10, 30, 0 ] ]

  - [ [ 10, 20, 0 ], [ 10, 30, 0 ], [ 15, 30, 0 ] ]
  - [ [ 10, 20, 0 ], [ 15, 20, 0 ], [ 15, 30, 0 ] ]

  - [ [ 15, 20, 0 ], [ 15, 30, 0 ], [ 20, 30, 0 ] ]
  - [ [ 15, 20, 0 ], [ 20, 20, 0 ], [ 20, 30, 0 ] ]

  - [ [ 0, 30, 0 ], [ 0, 40, 0 ], [ 5, 40, 0 ] ]
  - [ [ 0, 30, 0 ], [ 5, 30, 0 ], [ 5, 40, 0 ] ]

  - [ [ 5, 30, 0 ], [ 5, 40, 0 ], [ 10, 40, 0 ] ]
  - [ [ 5, 30, 0 ], [ 10, 30, 0 ], [ 10, 40, 0 ] ]

  - [ [ 10, 30, 0 ], [ 10, 40, 0 ], [ 15, 40, 0 ] ]
  - [ [ 10, 30, 0 ], [ 15, 30, 0 ], [ 15, 40, 0 ] ]

  - [ [ 15, 30, 0 ], [ 15, 40, 0 ], [ 20, 40, 0 ] ]
  - [ [ 15, 30, 0 ], [ 20, 30, 0 ], [ 20, 40, 0 ] ]


  # WALLS
  - [ [ 0, 10, 0 ], [ 5, 10, 0 ], [ 5, 10, 5 ] ]
  - [ [ 0, 10, 0 ], [ 0, 10, 5 ], [ 5, 10, 5 ] ]
  - [ [ 0, 20, 0 ], [ 5, 20, 0 ], [ 5, 20, 5 ] ]
  - [ [ 0, 20, 0 ], [ 0, 20, 5 ], [ 5, 20, 5 ] ]

  - [ [ 5, 10, 0 ], [ 5, 10, 5 ], [ 5, 20, 0 ],]
  - [ [ 5, 10, 5 ], [ 5, 20, 5 ], [ 5, 20, 0 ] ]

  - [ [0, 30, 0 ], [0, 40, 0 ], [0, 40, 5] ]
  - [ [0, 30, 0 ], [0, 30, 5 ], [0, 40, 5] ]

  - [ [0, 40, 0 ], [5, 40, 0 ], [5, 40, 5] ]
  - [ [0, 40, 0 ], [0, 40, 5 ], [5, 40, 5] ]

lines:
    - [[5, 0, 0], [5, 0, 5]]
    - [[10, 0, 0], [10, 0, 5]]
    - [[10, 10, 0], [10, 10, 5]]
    - [[10, 20, 0], [10, 20, 5]]
    - [[10, 30, 0], [10, 30, 5]]

    - [[15, 0, 0], [15, 0, 5]]
    - [[15, 10, 0], [15, 10, 5]]
    - [[15, 20, 0], [15, 20, 5]]
    - [[15, 30, 0], [15, 30, 5]]

spheres:
    - radius: 1.0
      center: [10, 0, 5]

    - radius: 0.5
      center: [10, 10, 5]

    - radius: 2.0
      center: [10, 20, 5]

    - radius: 0.2
      center: [10, 30, 5]

poses:
  - quaternion: [ 0, 0, 0, 1 ]
    translation: [ 2.5, 5, 2 ]
    dest_frame_id: 0
  - quaternion: [ 0.4, 0.5, 0.3, 1 ]
    translation: [ 7.5, 5, 2 ]
    dest_frame_id: 1
  - quaternion: [ 0.2, 0.7, 0.1, 1 ]
    translation: [ 7.5, 15, 3 ]
    dest_frame_id: 2
  - quaternion: [ 0.02, 0.3, 0.1, 1 ]
    translation: [ 7.5, 25, 2 ]
    dest_frame_id: 3
  - quaternion: [ -0.1, 0.4, 0.2, 1 ]
    translation: [ 2.5, 25, 2 ]
    dest_frame_id: 4
  - quaternion: [ -0.3, 0.06, 0.07, 1 ]
    translation: [ 2.5, 35, 1 ]
    dest_frame_id: 5
  - quaternion: [ 0.01, 0.02, 0.6, 1 ]
    translation: [ 7.5, 35, 1.5 ]
    dest_frame_id: 6
  - quaternion: [ 0.01, -0.07, 0.01, 1]
    translation: [ 12.5, 35, 2 ]
    dest_frame_id: 7
  - quaternion: [ 0, 0, 0.001, 1 ]
    translation: [ 12.5, 25, 2.5 ]
    dest_frame_id: 8
  - quaternion: [ -0.1, 0.8, 0.9, 0.1 ]
    translation: [ 12.5, 15, 2 ]
    dest_frame_id: 9
  - quaternion: [ 0, 0.2, 0.1, 1 ]
    translation: [ 12.5, 5, 3 ]
    dest_frame_id: 10
)";
    std::stringstream ss(scene_yaml);
    YAML::Node node = YAML::Load(ss);

    auto acquisition = slam::SyntheticSensorAcquisition::ReadYAML(node);

    auto points = acquisition.GetScene()->GeneratePoints(5000);

    auto &instance = viz3d::GUI::Instance();
    auto window = std::make_shared<slam::MultiPolyDataWindow>("Main Window");
    std::vector<slam::WPoint3D> wpoints(points.size());
    for (auto i(0); i < wpoints.size(); ++i)
        wpoints[i].WorldPoint() = points[i];
    auto poly_data = slam::polydata_from_points(wpoints);
    poly_data->GetPointData()->SetActiveScalars("Y");
    window->AddPolyData("Scene", 0, poly_data);
    window->InitializeWindow();

    std::thread gui_thread{viz3d::GUI::LaunchMainLoop, "GUI"};
    std::this_thread::sleep_for(std::chrono::duration<double, std::ratio<1, 1>>(2.0));
    instance.SignalClose();
    gui_thread.join();
#endif

}