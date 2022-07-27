#include <gtest/gtest.h>
#include <SlamCore/cereal.h>
#include <cereal/archives/json.hpp>
#include "test_utils.h"

TEST(CEREAL, conversions) {

#define CEREAL_COPY_AND_LOAD \
    std::stringstream ss; \
    { \
        cereal::JSONOutputArchive oa(ss); \
        oa(source); \
    } \
    { \
        cereal::JSONInputArchive ia(ss); \
        ia(copy); \
    }

    {
        Eigen::Matrix3d source, copy;
        source = Eigen::Matrix3d::Random();
        CEREAL_COPY_AND_LOAD
        ASSERT_EQUALS_MATRICES(copy, source);
    }

    {
        slam::SE3 source, copy;
        source = slam::SE3(Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random());
        CEREAL_COPY_AND_LOAD
        ASSERT_EQUALS_MATRICES(source.quat.coeffs(), copy.quat.coeffs());
    }

    {
        slam::WPoint3D source, copy;
        source.RawPoint() = Eigen::Vector3d::Random();
        source.WorldPoint() = Eigen::Vector3d::Random();
        source.Timestamp() = rand();
        source.index_frame = rand();
        CEREAL_COPY_AND_LOAD
        ASSERT_EQUALS_MATRICES(source.WorldPoint(), copy.WorldPoint());
        ASSERT_EQUALS_MATRICES(source.RawPoint(), copy.RawPoint());
        ASSERT_EQ(source.index_frame, copy.index_frame);
        ASSERT_EQ(source.Timestamp(), copy.Timestamp());
        ASSERT_EQ(source.Timestamp(), copy.Timestamp());
    }


    {
        slam::Pose source, copy;
        source.pose = {Eigen::Quaterniond::UnitRandom(), Eigen::Vector3d::Random()};
        source.dest_timestamp = rand();
        source.dest_frame_id = rand();
        source.ref_timestamp = rand();
        source.ref_frame_id = rand();
        CEREAL_COPY_AND_LOAD
        ASSERT_EQUALS_MATRICES(source.pose.quat.coeffs(), copy.pose.quat.coeffs());
        ASSERT_EQUALS_MATRICES(source.pose.tr, copy.pose.tr);
        ASSERT_EQ(source.ref_frame_id, copy.ref_frame_id);
        ASSERT_EQ(source.dest_frame_id, copy.dest_frame_id);
        ASSERT_EQ(source.dest_timestamp, copy.dest_timestamp);
        ASSERT_EQ(source.ref_timestamp, copy.ref_timestamp);
    }

}
