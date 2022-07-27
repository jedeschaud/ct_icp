#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <glog/logging.h>
#include <SlamCore/config_utils.h>

TEST(config, overrides) {

    std::string base_config_str = R"(
node1:
    value1: 1
    value2: "Toto"
    value3: "toto.titi.tata"
    given_pose:
        quat: [0.25, 0.25, 0.25, 0.33]
        tr: [1.0, 2.0, 3.0]
node2:
    child1:
        value1: 42
        child2:
            value2: 27
            value3: tata
            child3: hello
    list1:
        - 1
        - 2
        - 3
node3:
    toto: tata
)";
    std::stringstream ss_base(base_config_str);

    std::string overrides_str = R"(
node3:
    value: hello
    tata: toto

overrides:
    - node1.value1: 67
    - node2.value1: 12
    - node2.child1.child2.value2: 60
    - node2.child3.child4.value8: 33
    - node2.list1.[1]: 43
)";
    std::stringstream ss_overrides(overrides_str);

    auto base_node = YAML::Load(ss_base);
    auto overrides_node = YAML::Load(ss_overrides);

    ASSERT_EQ(slam::config::FindNode(overrides_node,
                                     "overrides.[3]", false)["node2.child3.child4.value8"].as<int>(), 33);
    slam::config::MergeOverridesNode(base_node, overrides_node);

    ASSERT_EQ((base_node["node1"])["value1"].as<int>(), 67);
    ASSERT_STREQ(base_node["node2"]["value1"].as<std::string>().c_str(), "12");
    ASSERT_EQ(base_node["node2"]["list1"][1].as<int>(), 43);
    ASSERT_EQ(base_node["node2"]["child1"]["child2"]["value2"].as<int>(), 60);
    ASSERT_EQ(base_node["node2"]["child3"]["child4"]["value8"].as<int>(), 33);

    YAML::Node pose_node = base_node["node1"]["given_pose"];
    auto pose = slam::config::PoseFromNode(pose_node);
    ASSERT_EQ(pose.tr.x(), 1.0);
    ASSERT_EQ(pose.tr.y(), 2.0);
    ASSERT_EQ(pose.tr.z(), 3.0);

    ASSERT_EQ(pose.quat.x(), 0.25);
    ASSERT_EQ(pose.quat.y(), 0.25);
    ASSERT_EQ(pose.quat.z(), 0.25);
    ASSERT_EQ(pose.quat.w(), 0.33);
}