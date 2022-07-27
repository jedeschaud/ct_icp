#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <SlamCore/config_utils.h>
#include <SlamCore/trajectory.h>
#include <iostream>


enum TEST_ENUM {
    V1 = 1,
    V2 = 10,
    V3 = 20
};

struct TestSTruct {

    int a = 1;
    double b = 2.0;
    std::string test = "hello tata";
    TEST_ENUM test_enum = V1;
    int x = 42;
};

TEST(config_utils, Conversion) {
    std::string yaml_str = R"(
a: 2
b: 3.0
test: "hello michael"
test_enum: V2
                           )";
    std::stringstream ss(yaml_str);
    YAML::Node node = YAML::Load(ss);
    TestSTruct option;

    FIND_REQUIRED_OPTION(node, option, a, int)
    FIND_OPTION(node, option, b, double)
    FIND_OPTION(node, option, test, std::string)
    FIND_OPTION(node, option, x, int)
    slam::config::FindEnumOption(node, (int &) (option.test_enum), "test_enum", {
            {"V1", TEST_ENUM::V1},
            {"V2", TEST_ENUM::V2},
            {"V3", TEST_ENUM::V3},
    });


    ASSERT_EQ(option.test_enum, TEST_ENUM::V2);
    ASSERT_EQ(option.a, 2);
    ASSERT_EQ(option.b, 3.0);
    ASSERT_EQ(option.x, 42);
    ASSERT_STREQ(option.test.c_str(), "hello michael");

//    auto yaml_node = slam::config::RootNode("/home/pdell/dev/LidarMapping/LidarBA/config/nclt_config.yaml");
//    auto ba_node = yaml_node["ba_options"];
//    auto num_iters = ba_node["num_iters"].as<int>();
//    ASSERT_EQ(num_iters, 3);

}



