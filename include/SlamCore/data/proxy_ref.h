#ifndef SLAMCORE_PROXY_REF_H
#define SLAMCORE_PROXY_REF_H

#include "SlamCore/data/schema.h"

namespace slam {

    /*!
     *  @brief   A Proxy Static Reference, allows seemless conversion between a reference of a given source type,
     *           And a destination type, using an instance of Conversion for the conversion
     *
     *  @note   Compared to ProxyDRef, the source value type is known at compile time
     */
    template<typename _DestValueT, typename _SourceValueT, class _ConversionT>
    struct ProxySRef {

        _SourceValueT &ref;
        _ConversionT conversion;

        explicit ProxySRef(_SourceValueT &reference) : ref(reference) {}

        ProxySRef(_SourceValueT &reference, _ConversionT _conversion) : ref(reference), conversion(_conversion) {}

        ProxySRef(const ProxySRef &other) : ref(const_cast<_SourceValueT &>(other.ref)), conversion(other.conversion) {}

        ProxySRef(ProxySRef &&other) : ref(other.ref), conversion(other.conversion) {}

        ProxySRef &operator=(const ProxySRef &other) {
            ref = other.ref;
            conversion = other.conversion;
            return *this;
        }

        ProxySRef &operator=(ProxySRef &&other) {
            ref = other.ref;
            conversion = other.conversion;
            return *this;
        }

        ProxySRef &operator=(const _DestValueT &dest) {
            ref = conversion(dest);
            return *this;
        }

        ProxySRef operator&() {
            return *this;
        }

        ProxySRef operator*() {
            return *this;
        }

        operator _DestValueT() const {
            return conversion(ref);
        }
    };


    /*!
     * @brief   The base template specification for a ProxyDRef (Proxy Dynamic Reference),
     *          which is a proxy to a typed reference of a different type than the assignment and copy operators provide
     *
     * @note    It is useful to allow views to interact with buffers of items with an interface of a different
     *          Type than the underlying data.
     */
    template<typename DestT, class Enable = void>
    struct ProxyDRef {

        operator DestT() const {
            throw std::runtime_error("Invalid ProxyDRef Conversion. Not Implemented or not replaced.");
        };

        void operator=(const DestT &value) {
            throw std::runtime_error("Invalid ProxyDRef Conversion. Not Implemented or not replaced.");
        };

        void *pointer;
        const slam::PROPERTY_TYPE source_type_;
    };


    /*!
     * @brief  A Template specialization of a ProxyDRef for numeric Scalar types
     */
    template<class DestScalarT>
    struct ProxyDRef<DestScalarT, std::enable_if_t<std::is_arithmetic_v<DestScalarT>>> {
    private:
        template<typename SrcT>
        inline DestScalarT ConvertScalar() const {
            return static_cast<DestScalarT>(*static_cast<const SrcT *>(pointer));
        }

        template<typename SrcT>
        inline void Assign(const DestScalarT &value) {
            *static_cast<SrcT *>(pointer) = static_cast<SrcT>(value);
        }

    public:
        // Statically copies the data to the destination type
        operator DestScalarT() const {
            if (slam::StaticPropertyType<DestScalarT>() == source_type_)
                return *static_cast<DestScalarT *>(pointer);
            switch (source_type_) {
                case slam::INT8:
                    return ConvertScalar<char>();
                case slam::UINT8:
                    return ConvertScalar<unsigned char>();
                case slam::INT16:
                    return ConvertScalar<short>();
                case slam::UINT16:
                    return ConvertScalar<unsigned short>();
                case slam::INT32:
                    return ConvertScalar<int>();
                case slam::UINT32:
                    return ConvertScalar<unsigned int>();
                case slam::INT64:
                    return ConvertScalar<long long int>();
                case slam::UINT64:
                    return ConvertScalar<unsigned long long int>();
                case slam::FLOAT64:
                    return ConvertScalar<double>();
                case slam::FLOAT32:
                    return ConvertScalar<float>();
                default:
                    throw std::runtime_error("Not implemented error");
            }
        }


        // Assigns a scalar of a destination type, with a static casting to the underlying data type
        void operator=(const DestScalarT &value) {
            if (slam::StaticPropertyType<DestScalarT>() == source_type_) {
                *static_cast<DestScalarT *>(pointer) = value;
                return;
            }
            switch (source_type_) {
                case slam::FLOAT64:
                    Assign<double>(value);
                    break;
                case slam::FLOAT32:
                    Assign<float>(value);
                    break;
                case slam::INT8:
                    Assign<char>(value);
                    break;
                case slam::UINT8:
                    Assign<unsigned char>(value);
                    break;
                case slam::INT16:
                    Assign<short>(value);
                    break;
                case slam::UINT16:
                    Assign<unsigned short>(value);
                    break;
                case slam::INT32:
                    Assign<int>(value);
                    break;
                case slam::UINT32:
                    Assign<unsigned int>(value);
                    break;
                case slam::INT64:
                    Assign<long long int>(value);
                    break;
                case slam::UINT64:
                    Assign<unsigned long long int>(value);
                    break;
                default:
                    throw std::runtime_error("Not implemented error");
            }
        }

        ProxyDRef<DestScalarT, std::enable_if_t<std::is_arithmetic_v<DestScalarT>>>(void *ptr,
                                                                                    slam::PROPERTY_TYPE type)
                :
                pointer(ptr), source_type_(type) {}

        void *pointer;
        const slam::PROPERTY_TYPE source_type_;
    };


    /*!
     * @brief   Template specialization of a ProxyDRef for a std::array
     */
    template<typename DestScalarT, size_t _Size>
    struct ProxyDRef<std::array<DestScalarT, _Size>,
            std::enable_if_t<std::is_arithmetic_v<DestScalarT>>> {
    private:
        template<typename SrcT>
        inline std::array<DestScalarT, _Size> ConvertArray() const {
            std::array<DestScalarT, _Size> result;
            const auto &ptr = *reinterpret_cast<std::array<SrcT, _Size> *> (pointer);
            for (auto i(0); i < _Size; ++i)
                result[i] = ptr[i];
            return result;
        }

        template<typename SrcT>
        inline void AssignArray(const std::array<DestScalarT, _Size> &value) const {
            auto &ptr = reinterpret_cast<std::array<SrcT, _Size> *>(pointer);
            for (auto i(0); i < _Size; ++i)
                ptr[i] = value[i];
        }

    public:
        // Statically copies the data to the destination type
        operator std::array<DestScalarT, _Size>() const {
            if (slam::StaticPropertyType<DestScalarT>() == source_type_)
                return *static_cast<std::array<DestScalarT, _Size> *>(pointer);
            switch (source_type_) {
                case slam::INT8:
                    return ConvertArray<char>();
                case slam::UINT8:
                    return ConvertArray<unsigned char>();
                case slam::INT16:
                    return ConvertArray<short>();
                case slam::UINT16:
                    return ConvertArray<unsigned short>();
                case slam::INT32:
                    return ConvertArray<int>();
                case slam::UINT32:
                    return ConvertArray<unsigned int>();
                case slam::INT64:
                    return ConvertArray<long long int>();
                case slam::UINT64:
                    return ConvertArray<unsigned long long int>();
                case slam::FLOAT32:
                    return ConvertArray<float>();
                case slam::FLOAT64:
                    return ConvertArray<double>();
                default:
                    throw std::runtime_error("Not implemented error");
            }
        }

        // Statically copies the data to the destination type
        void operator=(const std::array<DestScalarT, _Size> &value) {
            if (slam::StaticPropertyType<DestScalarT>() == source_type_) {
                *static_cast<std::array<DestScalarT, _Size> *>(pointer) = value;
                return;
            }
            switch (source_type_) {
                case slam::INT8:
                    AssignArray<char>(value);
                    break;
                case slam::UINT8:
                    AssignArray<unsigned char>(value);
                    break;
                case slam::INT16:
                    AssignArray<short>(value);
                    break;
                case slam::UINT16:
                    AssignArray<unsigned short>(value);
                    break;
                case slam::INT32:
                    AssignArray<int>(value);
                    break;
                case slam::UINT32:
                    AssignArray<unsigned int>(value);
                    break;
                case slam::INT64:
                    AssignArray<long long int>(value);
                    break;
                case slam::UINT64:
                    AssignArray<unsigned long long int>(value);
                    break;
                case slam::FLOAT32:
                    AssignArray<float>(value);
                    break;
                case slam::FLOAT64:
                    AssignArray<double>(value);
                    break;
                default:
                    throw std::runtime_error("Not implemented error");
            }
        }

        void *pointer;
        const slam::PROPERTY_TYPE source_type_;
    };


    /*!
     * @brief   Template specialization of a ProxyDRef for an Eigen::Matrix of floating points Scalars
     */
    template<typename ScalarT, int _Rows, int _Cols>
    struct ProxyDRef<Eigen::Matrix<ScalarT, _Rows, _Cols>,
            std::enable_if_t<std::is_floating_point_v<ScalarT>>> {
        using MatrixT = Eigen::Matrix<ScalarT, _Rows, _Cols>;
        template<typename SrcT>
        using MatrixSrc = Eigen::Matrix<SrcT, _Rows, _Cols>;

        // Returns a copy of the matrix stored in the reference, cast to the return type
        operator MatrixT() const {
            if (slam::StaticPropertyType<ScalarT>() == source_type_)
                return *static_cast<MatrixSrc<ScalarT> *>(pointer);
            switch (source_type_) {
                case slam::FLOAT64:
                    return static_cast<MatrixSrc<double> *>(pointer)->template cast<ScalarT>();
                case slam::FLOAT32:
                    return static_cast<MatrixSrc<float> *>(pointer)->template cast<ScalarT>();
                default:
                    throw std::runtime_error("Not implemented error");
            }
        }

        // Assigns data to the matrix stored in the reference, after casting the argument to the desired type
        void operator=(const MatrixT &value) {
            if (slam::StaticPropertyType<ScalarT>() == source_type_) {
                *static_cast<MatrixSrc<ScalarT> *>(pointer) = value;
                return;
            }
            switch (source_type_) {
                case slam::FLOAT64:
                    *static_cast<MatrixSrc<double> *>(pointer) = value.template cast<double>();
                    break;
                case slam::FLOAT32:
                    *static_cast<MatrixSrc<float> *>(pointer) = value.template cast<float>();
                    break;
                default:
                    throw std::runtime_error("Not implemented error");
            }
        }

        void *pointer;
        const slam::PROPERTY_TYPE source_type_;
    };

} // namespace

#endif //SLAMCORE_PROXY_REF_H
