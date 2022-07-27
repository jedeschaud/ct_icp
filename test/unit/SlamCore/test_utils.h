#ifndef SlamCore_TEST_UTILS_H
#define SlamCore_TEST_UTILS_H

namespace test {

    /* -------------------------------------------------------------------------------------------------------------- */

    // Generate random data for a buffer of given size
    inline void random_data(unsigned char *data_ptr, size_t size) {
        for (auto i(0); i < size; ++i)
            data_ptr[i] = static_cast<unsigned char>(rand() % 255);
    };

    // Returns whether two matrices are equal (up to a threshold)
    template<typename _ScalarT, int _Rows, int _Cols>
    inline bool is_equal(const Eigen::Matrix<_ScalarT, _Rows, _Cols> &lhs,
                  const Eigen::Matrix<_ScalarT, _Rows, _Cols> &rhs,
                  double threshold = 0.) {
        return (lhs - rhs).norm() <= threshold;
    }

} // namespace test


#define ASSERT_EQUALS_MATRICES(mat1, mat2) ASSERT_TRUE(test::is_equal<>(mat1, mat2));


#endif //SlamCore_TEST_UTILS_H
