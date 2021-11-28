#include <gtest/gtest.h>
#include <ct_icp/dataset.h>
#include <thread>
#include <viz3d/engine.h>
#include <SlamUtils/viz3d_utils.h>

TEST(ct_icp, PLYDirectory) {

    std::thread gui_thread{viz::ExplorationEngine::LaunchMainLoop};
    auto root_path = "/media/pdell/SSD_1/DATASETS/KITTI-360/00/frames";
    auto directory = ct_icp::PLYDirectory::FromDirectoryPath(root_path);
    directory.Schema().timestamp_element_and_property = {"vertex", "timestamp"};

    while (directory.HasNext()) {
        auto next = directory.NextUnfilteredFrame();

        auto &instance = viz::ExplorationEngine::Instance();
        auto model_ptr = std::make_shared<viz::PointCloudModel>();
        auto &model_data = model_ptr->ModelData();
        model_data.xyz = slam::slam_to_viz3d_pc(next.points, false);
        model_data.rgb = slam::get_field_color(next.points, slam::JET);

        instance.AddModel(0, model_ptr);
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1.));
    }

    gui_thread.join();
}