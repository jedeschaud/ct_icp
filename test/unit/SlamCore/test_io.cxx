#include <fstream>
#include <gtest/gtest.h>

#include "SlamCore/types.h"
#include "SlamCore/generic_tools.h"
#include "SlamCore/io.h"
#include "SlamCore/pointcloud.h"

/* ------------------------------------------------------------------------------------------------------------------ */
// Read / Write PLY File
TEST(io, Read_WritePLYFile_WPoint3D) {
    std::vector<slam::WPoint3D> points(10);

    for (auto &point: points) {
        point.RawPoint() = Eigen::Vector3d::Random();
        point.Timestamp() = (double) rand() / RAND_MAX;
    }
    auto pc = slam::PointCloud::WrapVector(points, slam::WPoint3D::DefaultSchema(), "raw_point");

    std::stringstream ss_stream;
    auto schema = slam::WPoint3D::DoubleSchemaMapper();
    schema.XYZElementName() = "world_point";
    slam::WritePLY(ss_stream, pc, schema);
    auto pc_copy = slam::ReadWPoint3DVectorFromPLYStream(ss_stream, schema);

    ASSERT_EQ(pc_copy.size(), points.size());
    for (int i(0); i < points.size(); ++i) {
        double norm = (pc_copy[i].RawPointConst() - points[i].RawPoint()).norm();
        ASSERT_EQ(norm, 0.);
        ASSERT_EQ(pc_copy[i].Timestamp(), points[i].Timestamp());
    }

    std::stringstream ss_stream2;
    auto schema2 = slam::PLYSchemaMapper::BuildDefaultFromBufferCollection(pc.GetCollection());
    schema2.XYZElementName() = "raw_point";
    slam::WritePLY(ss_stream2, pc, schema2);
    auto pc_copy2 = slam::ReadWPoint3DVectorFromPLYStream(ss_stream2, schema2);

    ASSERT_EQ(pc_copy2.size(), points.size());
    for (int i(0); i < points.size(); ++i) {
        double norm = (pc_copy2[i].RawPointConst() - points[i].RawPoint()).norm();
        ASSERT_EQ(norm, 0.);
        ASSERT_EQ(pc_copy2[i].Timestamp(), points[i].Timestamp());
    }

    // Load a Default PLY from file
    std::stringstream ss_stream3;
    slam::WritePLY(ss_stream2, pc, schema2);
    auto pc_default = slam::ReadPointCloudFromPLY(ss_stream);

    auto num_items = pc_default->GetCollection().NumItemsInSchema();
    for (auto idx(0); idx < num_items; ++idx) {
        std::cout << pc_default->GetCollection().GetItemInfo(idx).item_schema << std::endl;
    }

    auto view_points = pc_default->XYZ<double>();
    for (auto i(0); i < points.size(); ++i) {
        Eigen::Vector3d pt = view_points[i];
        Eigen::Vector3d pt2 = pc_copy2[i].RawPoint();

        double diff = (pt - pt2).norm();
        ASSERT_EQ(diff, 0.);
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Read / Write poses as PLY File
TEST(io, Read_Write_Poses_As_PLY) {
    std::vector<slam::Pose> poses(10);
    for (auto &pose: poses) {
        pose.pose.quat = Eigen::Quaterniond::UnitRandom();
        pose.pose.tr = Eigen::Vector3d::Random();
        pose.dest_frame_id = static_cast<slam::frame_id_t >(rand());
        pose.dest_timestamp = (double) rand() / RAND_MAX;
        pose.ref_frame_id = static_cast<slam::frame_id_t >(rand());
        pose.ref_timestamp = (double) rand() / RAND_MAX;
    }

    std::stringstream ss_stream;
    slam::SavePosesAsPLY(ss_stream, poses);
    auto copy_poses = slam::ReadPosesFromPLY(ss_stream);

    for (auto i(0); i < poses.size(); ++i) {
        auto &pose = poses[i];
        auto &copy = copy_poses[i];
        ASSERT_EQ(pose.dest_timestamp, copy.dest_timestamp);
        ASSERT_EQ(pose.dest_frame_id, copy.dest_frame_id);
        ASSERT_EQ(pose.ref_timestamp, copy.ref_timestamp);
        ASSERT_EQ(pose.ref_frame_id, copy.ref_frame_id);
        ASSERT_LE((pose.pose.Matrix() - copy.pose.Matrix()).cwiseAbs().maxCoeff(), 1.e-10);
    }
}