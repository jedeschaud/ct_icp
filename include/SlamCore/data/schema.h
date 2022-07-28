#ifndef SlamCore_SCHEMA_H
#define SlamCore_SCHEMA_H

#include <map>
#include <optional>

#include <Eigen/Dense>
#include <glog/logging.h>

namespace slam {

    /*!
     * @brief   Base property types for a point cloud
     */
    enum PROPERTY_TYPE {
        FLOAT32,
        FLOAT64,
        INT8,
        UINT8,
        INT16,
        UINT16,
        INT32,
        UINT32,
        INT64,
        UINT64
    };

    /*!
     * @brief   Returns a char representing a property type
     */
    inline char PropertyTypeChar(PROPERTY_TYPE type) {
        switch (type) {
            case FLOAT32:
                return 'f';
            case FLOAT64:
                return 'd';
            case INT8:
                return 'b';
            case UINT8:
                return 'B';
            case INT16:
                return 's';
            case UINT16:
                return 'S';
            case INT32:
                return 'i';
            case UINT32:
                return 'I';
            case INT64:
                return 'l';
            case UINT64:
                return 'L';
        }
        throw std::runtime_error("The property type " + std::to_string(type) + " does not exist");
    }

    template<PROPERTY_TYPE type>
    constexpr int StaticPropertySize() {
        if constexpr(type == FLOAT32)
            return sizeof(float);
        if constexpr(type == FLOAT64)
            return sizeof(double);
        if constexpr(type == INT8 || type == UINT8)
            return sizeof(char);
        if constexpr(type == INT16 || type == UINT16)
            return sizeof(short);
        if constexpr(type == INT32 || type == UINT32)
            return sizeof(int);
        if constexpr(type == INT64 || type == UINT64)
            return sizeof(long long int);
    }

    /*!
     * @brief   Returns the size of a Property
     */
    inline int PropertySize(PROPERTY_TYPE type) {
        switch (type) {
            case FLOAT32:
                return StaticPropertySize<FLOAT32>();
            case FLOAT64:
                return StaticPropertySize<FLOAT64>();
            case INT8:
            case UINT8:
                return StaticPropertySize<INT8>();
            case INT16:
            case UINT16:
                return StaticPropertySize<INT16>();
            case INT32:
            case UINT32:
                return StaticPropertySize<INT32>();
            case INT64:
            case UINT64:
                return StaticPropertySize<INT64>();
        }
        throw std::runtime_error("The property type " + std::to_string(type) + " does not exist");
    }

    /*!
     * @brief   Returns the property type associated to the template parameter.
     *
     * @tparam  ScalarT     A Template parameter
     */
    template<typename ScalarT>
    inline PROPERTY_TYPE StaticPropertyType() {
        static_assert(std::is_arithmetic_v<ScalarT>);
        if constexpr(std::is_same_v<ScalarT, double>)
            return FLOAT64;
        if constexpr(std::is_same_v<ScalarT, float>)
            return FLOAT32;
        if constexpr(std::is_same_v<ScalarT, char>)
            return INT8;
        if constexpr(std::is_same_v<ScalarT, unsigned char>)
            return UINT8;
        if constexpr(std::is_same_v<ScalarT, short>)
            return INT16;
        if constexpr(std::is_same_v<ScalarT, unsigned short>)
            return UINT16;
        if constexpr(std::is_same_v<ScalarT, int>)
            return INT32;
        if constexpr(std::is_same_v<ScalarT, long int>) {
            // System dependent (cf LP32 / ILP32 / LLP64 / LP64)
            // cf https://en.cppreference.com/w/cpp/language/types
            if (sizeof(long int) == sizeof(long long int))
                return INT64;
            return INT32;
        }
        if constexpr(std::is_same_v<ScalarT, unsigned long int>) {
            // See above comment
            if (sizeof(long int) == sizeof(unsigned long long int))
                return UINT64;
            return UINT32;
        }
        if constexpr(std::is_same_v<ScalarT, unsigned int>)
            return UINT32;
        if constexpr(std::is_same_v<ScalarT, long long>)
            return INT64;
        if constexpr(std::is_same_v<ScalarT, unsigned long long>)
            return UINT64;
        throw std::runtime_error("Type not supported");
    }

    // Forward Declarations
    struct ElementInfo;
    struct ItemInfo;
    struct ItemBuffer;

    /*!
     * @brief   A PropertyInfo defines the layout of a named property in an element.
     */
    struct PropertyInfo {
        int dimension = 1;              // Dimension of the property, ie the number of scalars defining the property vector
        int offset_in_elem = 0;         // Offset of the property in its element
        PROPERTY_TYPE type;             // Data Type of the element
        std::string property_name;      // Name of the property
        ElementInfo *parent = nullptr;  // Pointer to the parent ElementInfo

        inline int Size() const { return PropertySize(type) * dimension; }
    };

    /*!
     * @brief   An ElementInfo defines the layout of an element in an item.
     *
     * An Element wraps of a set of properties, which are laid contiguously in memory.
     *
     * Note:    `CheckMemoryLayout` will detect some layout inconsistencies between the properties
     *          But not all layout errors can be detected.
     */
    struct ElementInfo {
        std::vector<PropertyInfo> properties;     // Properties of the item info
        int offset_in_item = 0;                   // Offset of the element in its item
        std::string element_name;                 // Name of the element
        ItemInfo *parent = nullptr;               // Pointer to the parent ItemInfo

        int ElementSize() const;

        // Whether the point cloud has the given property name
        bool HasProperty(const std::string &property_name) const;

        // Returns the property with the given name
        const PropertyInfo &GetProperty(const std::string &property_name) const;

        // Checks that the memory layout is contiguous, throws an exception if it is not the case
        void CheckMemoryLayout() const;

        ElementInfo() = default;

        explicit ElementInfo(std::vector<PropertyInfo> &&_properties,
                             int _offset_in_item,
                             std::string &&_element_name);

    };


    /*!
     * @brief   An ItemSchema describes the memory layout of an Item
     */
    class ItemSchema {
    public:

        /*!
         * @brief   A builder class for the ItemSchema
         */
        class Builder {
        public:

            Builder() = default;

            explicit Builder(size_t item_size);

            Builder(const std::map<std::string, ElementInfo> &element_infos, int item_size);;

            // Returns the item size
            inline int GetItemSize() const { return item_size_; }

            // Returns an element stored in the builder
            slam::ElementInfo &GetElement(const std::string &element_name);

            // Sets the item size
            Builder &SetItemSize(int item_size);;

            // Add an element to the builder
            Builder &AddElement(const std::string &element_name, int offset_in_item);

            // Removes an element from the builder
            Builder &RemoveElement(const std::string &element_name);


            // Adds a property to the builder
            Builder &AddProperty(const std::string &element_name,
                                 std::string &&property_name,
                                 slam::PROPERTY_TYPE ptype,
                                 int offset_in_elem,
                                 int dim);

            // Adds a property to the builder
            template<typename ScalarT>
            Builder &AddScalarProperty(const std::string &element_name,
                                       std::string &&property_name,
                                       int offset_in_elem,
                                       int dim = 1);

            // Builds and returns the ItemSchema, after checking if it is valid
            ItemSchema Build() const;

        private:
            int item_size_ = -1;
            std::map<std::string, slam::ElementInfo> element_infos_;
        };

        // Returns a Builder initialized with the schema of the current item
        Builder GetBuilder() const;

        // Returns whether an element name exists in the Schema
        bool HasElement(const std::string &name) const;

        // Returns an ElementInfo for a given name
        const ElementInfo &GetElementInfo(const std::string &name) const {
            return element_infos_.at(name);
        }

        // Returns a vector of the element names
        std::vector<std::string> GetElementNames() const;

        // Returns whether the schema has a given property
        bool HasProperty(const std::string &element_name, const std::string &property_name) const;

        // Returns whether the schema has a given property
        const PropertyInfo &GetPropertyInfo(const std::string &element_name, const std::string &property_name) const;

        // Returns the item size
        inline int GetItemSize() const { return item_size_; }

        // Returns the sum of the elements size in the schema
        // Note: It should be smaller or equal to the item size
        int GetTotalElementSize() const;;

        ItemSchema(ItemSchema &&schema) {
            element_infos_ = std::move(schema.element_infos_);
            item_size_ = schema.item_size_;
        }

        ItemSchema() = default;

        ItemSchema(const ItemSchema &item_schema) = default;

        ItemSchema &operator=(const ItemSchema &) = default;

        ItemSchema &operator=(ItemSchema &&) = default;

        // Print the Schema in an ostream
        friend std::ostream &operator<<(std::ostream &os, const ItemSchema &schema);

    private:
        std::map<std::string, ElementInfo> element_infos_;
        int item_size_;

        ItemSchema(const std::map<std::string, ElementInfo> &element_infos, int item_size);

        friend class ItemInfo;

    };


    /*!
     * @brief   Creates a SchemaBuilder for an item containing a single element
     *
     * @note    This is a convenient method for defining schemas of Custom types via template specialization
     *          See below the template specialization for Eigen Vectors and std arrays
     */
    template<typename DataT>
    static ItemSchema::Builder BuilderFromSingleElementData(const std::string &element_name,
                                                            std::optional<std::vector<std::string>> properties = {}) {
        throw std::runtime_error("Not Implemented Exception");
    };

    /*!
     * @brief   An ItemInfo defines the layout of an item in memory.
     *
     * An item wraps of a set of elements. Each element is contained in the memory layout of an item.
     * Items can be larger than the size of the sum of its elements, but elements must not overlap.
     */
    struct ItemInfo {
        ItemSchema item_schema;                         // The layout information of the elements in the item
        ItemBuffer *parent_buffer = nullptr;            // A pointer to the parent ItemBuffer
        int item_size;                                  // The size in bytes of the contiguous item in memory

        bool HasElement(const std::string &element_name) const;

        explicit ItemInfo(ItemSchema &&schema, ItemBuffer &buffer, int item_size);

    private:
        friend class ItemBuffer;

        explicit ItemInfo() = default;

    };


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
#define SCHEMA_EIGEN_MATRIX_SPECIALIZATION(ScalarT, _Rows, _Cols) \
    template<> \
    ItemSchema::Builder BuilderFromSingleElementData<Eigen::Matrix<ScalarT, _Rows, _Cols>>(const std::string &element_name, \
                                                                                    std::optional<std::vector<std::string>> properties) { \
        static_assert(_Rows > 0 && _Cols > 0); \
        using MatT = Eigen::Matrix<ScalarT, _Rows, _Cols>; \
        ItemSchema::Builder builder(sizeof(MatT)); \
        builder.AddElement(element_name, 0); \
        const int kVectorDim = _Rows * _Cols; \
        if (properties) { \
            CHECK(properties->size() == kVectorDim) << "The dimension of the eigen vector is not compatible"; \
            int offset = 0; \
            for (auto &pname: *properties) { \
                builder.template AddScalarProperty<ScalarT>(element_name, std::string(pname), offset, 1); \
                offset += sizeof(ScalarT); \
            } \
        } else \
            builder.AddProperty(element_name, std::string(element_name), StaticPropertyType<ScalarT>(), 0, kVectorDim); \
        return builder; \
    };

    SCHEMA_EIGEN_MATRIX_SPECIALIZATION(float, 3, 1)

    SCHEMA_EIGEN_MATRIX_SPECIALIZATION(float, 4, 1)

    SCHEMA_EIGEN_MATRIX_SPECIALIZATION(float, 3, 3)

    SCHEMA_EIGEN_MATRIX_SPECIALIZATION(float, 4, 4)

    SCHEMA_EIGEN_MATRIX_SPECIALIZATION(double, 3, 1)

    SCHEMA_EIGEN_MATRIX_SPECIALIZATION(double, 4, 1)

    SCHEMA_EIGEN_MATRIX_SPECIALIZATION(double, 3, 3)

    SCHEMA_EIGEN_MATRIX_SPECIALIZATION(double, 4, 4)


    /* -------------------------------------------------------------------------------------------------------------- */
#define SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION(ScalarT, Size)                                                           \
    template<>                                                                                                          \
    ItemSchema::Builder BuilderFromSingleElementData<std::array<ScalarT, Size>>(const std::string &element_name,                     \
                                                                   std::optional<std::vector<std::string>> properties) {\
        using ArrayT = std::array<ScalarT, Size>;                                                                       \
        ItemSchema::Builder builder(sizeof(ArrayT));                                                                                \
        builder.AddElement(element_name, 0);                                                                            \
        if (properties) {                                                                                               \
            CHECK(properties->size() == Size) << "The dimension of the eigen vector is not compatible";                 \
            int offset = 0;                                                                                             \
            for (auto &pname: *properties) {                                                                            \
                builder.template AddScalarProperty<ScalarT>(element_name, std::string(pname), offset, 1);               \
                offset += sizeof(ScalarT);                                                                              \
            }                                                                                                           \
        } else                                                                                                          \
            builder.AddProperty(element_name, std::string(element_name), StaticPropertyType<ScalarT>(),                 \
                                0, static_cast<int>(Size));                                                             \
                                                                                                                        \
        return builder;                                                                                                 \
    }

#define SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION_SMALL_VALUES(ScalarT)    \
    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION(ScalarT, 2)                  \
    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION(ScalarT, 3)                  \
    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION(ScalarT, 4)

    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION_SMALL_VALUES(double)

    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION_SMALL_VALUES(float)

    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION_SMALL_VALUES(char)

    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION_SMALL_VALUES(unsigned char)

    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION_SMALL_VALUES(int)

    SCHEMA_STD_ARRAY_MATRIX_SPECIALIZATION_SMALL_VALUES(unsigned int)


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename ScalarT>
    ItemSchema::Builder &
    ItemSchema::Builder::AddScalarProperty(const std::string &element_name, std::string &&property_name,
                                           int offset_in_elem, int dim) {
        static_assert(std::is_arithmetic_v<ScalarT>,
                      "The template parameter is not a fundamental arithmetic type");
        return AddProperty(element_name,
                           std::move(property_name),
                           slam::StaticPropertyType<ScalarT>(),
                           offset_in_elem, dim);
    }


} // namespace slam

#endif //SlamCore_SCHEMA_H
