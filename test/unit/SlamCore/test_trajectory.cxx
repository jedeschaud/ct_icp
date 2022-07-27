#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <SlamCore/types.h>
#include <SlamCore/trajectory.h>

TEST(DataTypesTest, test_definition) {

    std::vector<slam::Pose> poses;
    poses.push_back(slam::Pose(Eigen::Quaterniond::UnitRandom(),
                               Eigen::Vector3d::Random(),
                               0.0, slam::frame_id_t(0)));
    poses.push_back(slam::Pose(Eigen::Quaterniond::UnitRandom(),
                               Eigen::Vector3d::Random(),
                               0.5, 0));
    poses.push_back(slam::Pose(Eigen::Quaterniond::UnitRandom(),
                               Eigen::Vector3d::Random(),
                               0.9, 0));
    poses.push_back(slam::Pose(Eigen::Quaterniond::UnitRandom(),
                               Eigen::Vector3d::Random(),
                               1.01, 1));
    poses.push_back(slam::Pose(Eigen::Quaterniond::UnitRandom(),
                               Eigen::Vector3d::Random(),
                               1.3, 1));
    poses.push_back(slam::Pose(Eigen::Quaterniond::UnitRandom(),
                               Eigen::Vector3d::Random(),
                               1.5, 1));
    poses.push_back(slam::Pose(Eigen::Quaterniond::UnitRandom(),
                               Eigen::Vector3d::Random(),
                               2.0, 2));
    poses.push_back(slam::Pose(Eigen::Quaterniond::UnitRandom(),
                               Eigen::Vector3d::Random(),
                               3.0, 3));
    auto trajectory = slam::LinearContinuousTrajectory::Create(std::vector<slam::Pose>(poses));
    auto new_pose = trajectory.InterpolatePose(trajectory.MinTimestamp());
    auto pose = trajectory.InterpolatePose(1.0);
    auto pose_2 = trajectory.InterpolatePoseWithFrameId(1.1, 1);

    ASSERT_EQ(trajectory.MaxTimestamp(), 3.0);
    ASSERT_EQ(trajectory.MinTimestamp(), 0.0);

    ASSERT_FALSE(new_pose.Matrix().hasNaN());
    ASSERT_FALSE(pose.Matrix().hasNaN());

    auto rposes = trajectory.ToRelativePoses();
    auto new_trajectory = slam::LinearContinuousTrajectory::FromRelativePoses(std::move(rposes));

    auto &old_trajectory_poses = trajectory.ChangeReferenceFrame(trajectory.Poses()[0].Inverse()).Poses();
    auto &new_trajctory_poses = new_trajectory.Poses();
    ASSERT_EQ(old_trajectory_poses.size(), new_trajctory_poses.size());
    for (auto i(0); i < old_trajectory_poses.size(); ++i) {
        ASSERT_LE((old_trajectory_poses[i].Matrix() -
                   new_trajctory_poses[i].Matrix()).cwiseAbs().maxCoeff(), 1.e-10);
    }

    auto selection_traj = trajectory.SelectFramesWindowByFrameId(1, 2);
    ASSERT_EQ(selection_traj.Poses().size(), 4);

    auto selection_2 = trajectory.SelectFramesWindowByIndex(0, 0);


    std::vector<slam::Pose> _poses;
    auto get_pose = [](int frame_index) {
        return slam::Pose(Eigen::Quaterniond::UnitRandom(),
                          Eigen::Vector3d::Random(),
                          (double) frame_index, frame_index);
    };
    auto get_point = [](int frame_index, double timestamp) {
        slam::WPoint3D point;
        point.index_frame = frame_index;
        point.raw_point.timestamp = timestamp;
        return point;
    };
    _poses.push_back(get_pose(0));
    _poses.push_back(get_pose(1));
    _poses.push_back(get_pose(2));
    _poses.push_back(get_pose(3));
    auto _trajectory = slam::LinearContinuousTrajectory::Create(std::move(_poses));

    std::vector<slam::WPoint3D> _points;
    _points.push_back(get_point(0, 0));
    _points.push_back(get_point(0, 0.5));
    _points.push_back(get_point(0, 1));

    size_t l_pose_id, r_pose_id;
    _trajectory.GetClosestPosesIds(0, l_pose_id, r_pose_id);
    ASSERT_EQ(l_pose_id, 0);
    ASSERT_EQ(r_pose_id, 0);
    _trajectory.GetClosestPosesIds(0, 0, l_pose_id, r_pose_id);
    ASSERT_EQ(l_pose_id, 0);
    ASSERT_EQ(r_pose_id, 0);
    _trajectory.GetClosestPosesIds(0.5, 0, l_pose_id, r_pose_id);
    ASSERT_EQ(l_pose_id, 0);
    ASSERT_EQ(r_pose_id, 1);
    _trajectory.GetClosestPosesIds(1.0, 0, l_pose_id, r_pose_id);
    ASSERT_EQ(l_pose_id, 0);
    ASSERT_EQ(r_pose_id, 1);
    _trajectory.GetClosestPosesIds(1.0, 1, l_pose_id, r_pose_id);
    ASSERT_EQ(l_pose_id, 0);
    ASSERT_EQ(r_pose_id, 1);
}

TEST(Trajectory, FindClosestPose) {
    auto trajectory = slam::LinearContinuousTrajectory::Create({
                                                                       slam::Pose(slam::SE3(), 0, 0),
                                                                       slam::Pose(slam::SE3(), 1, 1),
                                                                       slam::Pose(slam::SE3(), 2, 2)
                                                               });
    size_t lb, lu;
    trajectory.GetClosestPosesIds(1.0, 1, lb, lu, false);
    ASSERT_EQ(lb, 0);
    ASSERT_EQ(lu, 1);
    trajectory.GetClosestPosesIds(2.0, 2, lb, lu, false);
    ASSERT_EQ(lb, 1);
    ASSERT_EQ(lu, 2);
    trajectory.GetClosestPosesIds(1.5, 2, lb, lu, false);
    ASSERT_EQ(lb, 1);
    ASSERT_EQ(lu, 2);
    trajectory.GetClosestPosesIds(1.5, 1, lb, lu, false);
    ASSERT_EQ(lb, 1);
    ASSERT_EQ(lu, 2);
    trajectory.GetClosestPosesIds(1.0, 0, lb, lu, false);
    ASSERT_EQ(lb, 0);
    ASSERT_EQ(lu, 1);


}

