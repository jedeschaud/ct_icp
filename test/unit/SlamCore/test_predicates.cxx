#include <gtest/gtest.h>

#include <map>

#include <Eigen/Dense>

#include <SlamCore/pointcloud.h>
#include <SlamCore/data/view.h>
#include "SlamCore/types.h"
#include <SlamCore/predicates.h>


TEST(Predicates, test) {
    typedef std::tuple<int, double, float> tuple_t;
    slam::TupleComparator<0> comparator;
    ASSERT_TRUE(comparator(tuple_t{1, 2., 3.f}, {2, 2., 3.f}));
    ASSERT_FALSE(comparator(tuple_t{1, 2., 3.f}, {1, 2., 3.f}));

    slam::TupleComparator<1> comparator_bis;
    ASSERT_TRUE(comparator_bis(std::pair<int, double>{1, 2.}, {1, 2.4}));

    //  Compile time error: cannot find a std::get overloading for a std::pair
    //    slam::TupleComparator<2> comparator_3;
    //    ASSERT_TRUE(comparator_3(std::pair<int, double>{1, 2.}, {1, 2.4}));
}