#include <gtest/gtest.h>
#include "SlamCore/traits.h"

TEST(traits, convert_pointer_trait) {
    using namespace slam;

    auto b1 = std::is_same_v<convert_pointer<const double *, float>::type, const float *>;
    auto b2 = std::is_same_v<convert_pointer<const double *, float>::type, float *>;
    auto b3 = std::is_same_v<convert_pointer<double *, float>::type, float *>;
    auto b4 = std::is_same_v<convert_pointer<double *, float>::type, const float *>;
    ASSERT_TRUE(b1);
    ASSERT_FALSE(b2);
    ASSERT_TRUE(b3);
    ASSERT_FALSE(b4);

    auto b5 = std::is_same_v<convert_pointer<std::vector<double>::iterator, float>::type, float *>;
    ASSERT_TRUE(b5);
}

