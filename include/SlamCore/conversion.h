#ifndef SLAMCORE_CONVERSION_H
#define SLAMCORE_CONVERSION_H

#include <Eigen/Dense>
#include "SlamCore/types.h"

namespace slam {

    struct conversion_tag {
    };

    struct reference_conversion_tag : conversion_tag {
    };

    struct deep_copy_conversion_tag : conversion_tag {
    };


    template<typename _SourceValueT, typename _ValueT, typename _ConversionTag>
    struct conversion_traits {
    };

    template<typename _SourceValueT, typename _ValueT>
    struct conversion_traits<_SourceValueT, _ValueT, reference_conversion_tag> {
        typedef _SourceValueT source_value_type;
        typedef source_value_type &source_reference;
        typedef const source_value_type &source_const_reference;
        typedef _ValueT value_type;
        typedef value_type &value_reference;
        typedef const value_type &value_const_reference;
        typedef reference_conversion_tag conversion_category;
    };

    template<typename _SourceValueT, typename _ValueT>
    struct conversion_traits<_SourceValueT, _ValueT, deep_copy_conversion_tag> {
        typedef _SourceValueT source_value_type;
        typedef _ValueT value_type;
        typedef deep_copy_conversion_tag conversion_category;
    };

#define REFERENCE_CONVERSION_TYPDEFS(source_t, dest_t) \
    private: \
    typedef conversion_traits< source_t , dest_t, reference_conversion_tag> __conversion_traits; \
    public: \
    typedef __conversion_traits::value_type value_type; \
    typedef __conversion_traits::value_reference value_reference; \
    typedef __conversion_traits::value_const_reference value_const_reference; \
    typedef __conversion_traits::source_value_type source_value_type; \
    typedef __conversion_traits::source_reference source_reference; \
    typedef __conversion_traits::source_const_reference source_const_reference; \
    typedef __conversion_traits::conversion_category conversion_category;

#define REFERENCE_CONVERSION_TYPDEFS_TEMPLATED(source_t, dest_t) \
    private: \
    typedef conversion_traits< source_t , dest_t, reference_conversion_tag> __conversion_traits; \
    public: \
    typedef typename __conversion_traits::value_type value_type; \
    typedef typename __conversion_traits::value_reference value_reference; \
    typedef typename __conversion_traits::value_const_reference value_const_reference; \
    typedef typename __conversion_traits::source_value_type source_value_type; \
    typedef typename __conversion_traits::source_reference source_reference; \
    typedef typename __conversion_traits::source_const_reference source_const_reference; \
    typedef typename __conversion_traits::conversion_category conversion_category;


#define DEEP_COPY_CONVERSION_TYPEDEFS(source_t, dest_t) \
    private:    \
    typedef conversion_traits< source_t , dest_t, deep_copy_conversion_tag> __conversion_traits; \
    public: \
    typedef __conversion_traits::value_type value_type; \
    typedef __conversion_traits::source_value_type source_value_type; \
    typedef __conversion_traits::conversion_category conversion_category;

#define DEEP_COPY_CONVERSION_TYPEDEFS_TEMPLATED(source_t, dest_t) \
    private:    \
    typedef conversion_traits< source_t , dest_t, deep_copy_conversion_tag> __conversion_traits; \
    public: \
    typedef  typename __conversion_traits::value_type value_type; \
    typedef  typename __conversion_traits::source_value_type source_value_type; \
    typedef  typename __conversion_traits::conversion_category conversion_category;


    template<typename T>
    struct IdentityConversion {
    REFERENCE_CONVERSION_TYPDEFS_TEMPLATED(T, T)

        value_reference operator()(source_reference v) const {
            return v;
        }

        value_const_reference operator()(source_const_reference v) const {
            return v;
        }
    };

    // @brief   Converts a slam::WPoint3d into a reference to its world_point
    struct WorldPointConversion {
    REFERENCE_CONVERSION_TYPDEFS(slam::WPoint3D, Eigen::Vector3d)

        value_reference operator()(source_reference point) const {
            return point.WorldPoint();
        }

        value_const_reference operator()(source_const_reference point) const {
            return point.WorldPointConst();
        }
    };

    // @brief   Converts a slam::WPoint3d into a reference to its raw_point
    struct RawPointConversion {
    REFERENCE_CONVERSION_TYPDEFS(slam::WPoint3D, Eigen::Vector3d)

        value_const_reference &operator()(source_const_reference point) const {
            return point.RawPointConst();
        }

        value_reference &operator()(source_reference &point) const {
            return point.RawPoint();
        }
    };

    // @brief   Converts a slam::Pose into a reference to its slam::SE3 pose
    struct PoseConversion {
    REFERENCE_CONVERSION_TYPDEFS(slam::Pose, slam::SE3)

        value_const_reference operator()(source_const_reference point) const {
            return point.pose;
        }

        value_reference operator()(source_reference point) const {
            return point.pose;
        }
    };


    // @brief  Converts an Eigen Matrix to an eigen matrix of a different type (by making a deep copy)
    template<typename _SourceScalarT, typename _DestScalarT, int _Rows, int _Cols>
    struct EigenCopyConversion {
        typedef Eigen::Matrix<_SourceScalarT, _Rows, _Cols> SourceMatT;
        typedef Eigen::Matrix<_DestScalarT, _Rows, _Cols> DestMatT;
    DEEP_COPY_CONVERSION_TYPEDEFS_TEMPLATED(SourceMatT, DestMatT)

        source_value_type operator()(const value_type &mat) const {
            return mat.template cast<_SourceScalarT>();
        }

        template<typename __DestMatT, std::enable_if_t<
                std::is_same_v<__DestMatT, DestMatT> && !std::is_same_v<_DestScalarT, _SourceScalarT>>>
        value_type operator()(const source_value_type &mat) const {
            return mat.template cast<_DestScalarT>();
        }
    };

    // @brief  Converts a std::array to an array of a different type and same size (by making a deep copy)
    template<typename _SourceScalarT, typename _DestScalarT, int _Size>
    struct ArrayCopyConversion {
        typedef std::array<_SourceScalarT, _Size> SourceArrayT;
        typedef std::array<_DestScalarT, _Size> DestArrayT;

    DEEP_COPY_CONVERSION_TYPEDEFS_TEMPLATED(SourceArrayT, DestArrayT)

        SourceArrayT operator()(const DestArrayT &mat) const {
            SourceArrayT result;
            for (auto i(0); i < _Size; ++i)
                result[i] = _SourceScalarT(mat[i]);
            return result;
        }

        template<typename __DestMatT, std::enable_if_t<
                std::is_same_v<__DestMatT, DestArrayT> && !std::is_same_v<_DestScalarT, _SourceScalarT>>>
        DestArrayT operator()(const SourceArrayT &mat) const {
            DestArrayT result;
            for (auto i(0); i < _Size; ++i)
                result[i] = _DestScalarT(mat[i]);
            return result;
        }
    };


} // namespace slam

#endif //SLAMCORE_CONVERSION_H
