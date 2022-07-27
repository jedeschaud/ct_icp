#ifndef SlamCore_POINTCLOUD_H
#define SlamCore_POINTCLOUD_H

#include <list>

#include "SlamCore/data/buffer_collection.h"
#include "SlamCore/trajectory.h"

namespace slam {

    class PointCloud;

    typedef std::shared_ptr<PointCloud> PointCloudPtr;

    // Returns whether the element info represents a valid XYZ element
    bool IsValidXYZElement(const ElementInfo &info);


    /*!
     * @brief   A PointCloud is a structure representing a set of 3D points with a custom set of fields.
     *          Each point in the PointCloud share the same number of these fields.
     *
     * A PointCloud mostly offers `views` of the data. The actual data layout is managed by a `BufferCollection`
     * Fields can be inserted, and deleted, but each property must have the same number of items equals to the number of points.
     *
     * The data layout and schema can be changed (at the cost of some copy).
     *
     */
    class PointCloud {
    public:

        /**
         * @brief A PointCloud::Field can either be an item, and element or a property of an item schema
         *
         * It allows a more flexible logic in selecting and manipulating a `column` (or field) of the point cloud,
         * Without requiring to know whether it is a property, an element or an item below
         * Which requires to know the layout of the data
         */
        struct Field {
            int item_index = 0;
            std::optional<std::string> element_name{};
            std::optional<std::string> property_name{};

            inline bool IsItem() const { return !element_name.has_value() && !property_name.has_value(); }

            inline bool IsElement() const { return element_name.has_value() && !property_name.has_value(); }

            inline bool IsProperty() const { return element_name.has_value() && property_name.has_value(); }
        };

        template<typename T>
        View<T> FieldView(const Field &property);

        template<typename T>
        const View<T> FieldView(const Field &property) const;

        template<typename T>
        ProxyView<T> ProxyFieldView(const Field &property);

        template<typename T>
        const ProxyView<T> ProxyFieldView(const Field &property) const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Constructors and static factories
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        PointCloud(BufferCollection &&collection, std::string &&xyz_element) : collection_(std::move(collection)) {
            SLAM_CHECK_STREAM(collection_.HasElement(xyz_element),
                              "The Collection does not have the element " << xyz_element << " in the schema" <<
                                                                          collection_.GetItemInfo(0).item_schema);
            auto &element_info = collection_.GetElement(xyz_element);
            xyz_ = {
                    int(collection_.GetItemIndex(xyz_element)),
                    {xyz_element}, {}
            };
        };

        PointCloud(BufferCollection &&collection, Field &&xyz_field) :
                collection_(std::move(collection)), xyz_(xyz_field) {}

        // Returns an empty point cloud with a default XYZ floating point vector in its item schema
        template<typename ScalarT>
        PointCloud static DefaultXYZ();

        template<typename ScalarT>
        PointCloudPtr static DefaultXYZPtr();

        // Returns a pointcloud wrapping the data contained in data
        template<typename ItemT, typename Alloc_ = std::allocator<ItemT>>
        PointCloud static WrapVector(std::vector<ItemT, Alloc_> &data, ItemSchema &&schema,
                                     const std::string &xyz_element);

        template<typename ItemT, typename Alloc_ = std::allocator<ItemT>>
        const PointCloud static WrapConstVector(const std::vector<ItemT, Alloc_> &data, ItemSchema &&schema,
                                                const std::string &xyz_element);

        /**
         * @brief Makes an empty and resizable point cloud for a custom Point type
         */
        template<typename PointT, typename ScalarT = float>
        std::enable_if_t<std::is_same_v<decltype(PointT::xyz), Eigen::Matrix<ScalarT, 3, 1>>,
                slam::PointCloudPtr> static MakeEmptyPointCloud(std::optional<slam::ItemSchema> schema = {},
                                                                const std::string &xyz_element = "vertex");

        /**
         * @returns Register default fields from the schema
         */
        void RegisterFieldsFromSchema();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Geometry API
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * Sets a new XYZ Field in the point cloud
         */
        void SetXYZField(Field &&field);

        /**
         * @returns The XYZ field describing the XYZ data of the point cloud
         */
        const Field &GetXYZField();

        // Changes the XYZ element to point to another element in the buffer collection
        void ChangeXYZElement(const std::string &element_name);

        // Returns a ProxyView of Eigen::Vector3(d/f) to the XYZ field
        template<typename ScalarT>
        ProxyView<Eigen::Matrix<ScalarT, 3, 1>> XYZ();

        // Returns a const ProxyView of Eigen::Vector3(d/f) to the XYZ field
        template<typename ScalarT>
        const ProxyView<Eigen::Matrix<ScalarT, 3, 1>> XYZConst() const;

        // Returns the number of points in the point cloud
        size_t size() const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Optional but common point cloud fields
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        bool IsValidField(const std::optional<Field> &field,
                          int expected_dimension,
                          bool check_floating_point = false) const;

        /** @brief Adds an element column to the point cloud, allocating its own item buffer memory, and copying data
         */
        template<typename DataT, slam::PROPERTY_TYPE pty_type>
        Field AddElementField(const std::string &element_name, const std::vector<DataT> &data);

        /** @brief Adds an empty element column to the point cloud, allocating its own item buffer memory
         */
        template<typename DataT, slam::PROPERTY_TYPE pty_type>
        Field AddElementField(const std::string &element_name);

        /**
         * @returns Returns whether the field
         */
        bool HasField(const std::string &field_name) const;

        /**
         * @brief Registers a field from the existing point cloud data
         *
         * @note This function will verify that the field is valid
         */
        void RegisterField(std::string &&field_name, Field &&field, int dimension);

        /**
         * @returns Returns the field associated to the Field name
         */
        Field GetField(const std::string &field_name) const;

        /**
         * @returns A Map of all fields in the point cloud
         */
        std::map<std::string, Field> GetRegisteredFields() const;

        // Macro to register default / conventional fields
#define REGISTER_OPTIONAL_FIELD(name, element_name, default_type, property_type, check_floating_point)  \
        inline bool Has ## name  ()  const { return element_name .has_value(); }                        \
        private:                                                                                        \
            std::optional<Field> element_name;                                                          \
            int name ## Dimension() const { return sizeof(default_type) / PropertySize(property_type);};\
        public:                                                                                         \
        inline Field Get ## name ## Field() const {                                                     \
            SLAM_CHECK_STREAM(element_name .has_value(), "The field ##element_name is not defined !");  \
            return element_name .value();                                                               \
            }                                                                                           \
        inline bool IsValid ## name  () const {                                                         \
            return IsValidField(element_name, name ## Dimension(), check_floating_point);               \
        }                                                                                               \
        inline void Set ## name(const std::string &elem_name, const std::vector<default_type> &data) {  \
            element_name = AddElementField<default_type, property_type>(elem_name, data);               \
        }                                                                                               \
        template<typename DataT>                                                                        \
        View<DataT> name() {                                                                            \
            SLAM_CHECK_STREAM(element_name.has_value(), "The field ##element_name is not defined !");   \
            return FieldView<DataT>(*element_name);                                                     \
        }                                                                                               \
        template<typename ScalarT>                                                                      \
        const View<ScalarT> name() const {                                                              \
            SLAM_CHECK_STREAM(element_name.has_value(), "The field ##element_name is not defined !");   \
            return FieldView<ScalarT>(*element_name);                                                   \
        }                                                                                               \
        template<typename ScalarT>                                                                      \
        ProxyView<ScalarT> name ## Proxy() {                                                            \
            SLAM_CHECK_STREAM(element_name.has_value(), "The field ##element_name is not defined !");   \
            return ProxyFieldView<ScalarT>(*element_name);                                              \
        }                                                                                               \
        template<typename ScalarT>                                                                      \
        const ProxyView<ScalarT> name ## Proxy() const {                                                \
            SLAM_CHECK_STREAM(element_name.has_value(), "The field ##element_name is not defined !");   \
            return ProxyFieldView<ScalarT>(*element_name);                                              \
        }                                                                                               \
        void Set ## name ## Field(Field&& field ) {                                                     \
            SLAM_CHECK_STREAM(IsValidField(field, name ## Dimension()), "Invalid Field");               \
            element_name = field;                                                                       \
        }                                                                                               \
        void Remove ## name ## Field() {                                                                \
            element_name = {};                                                                          \
        }                                                                                               \
        void AddDefault ## name ## Field() {                                                            \
            SLAM_CHECK_STREAM(!HasField(#element_name), "The pointcloud already has the field " << #element_name);\
            Set ## name ## Field(AddElementField<default_type, property_type>(#element_name));          \
        }


    public:
        REGISTER_OPTIONAL_FIELD(Intensity, intensity, float, FLOAT32, false)

        REGISTER_OPTIONAL_FIELD(Timestamps, timestamps, double, FLOAT64, false)

        typedef Eigen::Vector3f Vector3f;

        REGISTER_OPTIONAL_FIELD(Normals, normals, Vector3f, FLOAT32, true)

        REGISTER_OPTIONAL_FIELD(RGB, rgb, Vector3f, FLOAT32, true)

        REGISTER_OPTIONAL_FIELD(RawPoints, raw_point, Vector3f, FLOAT32, true)

        REGISTER_OPTIONAL_FIELD(WorldPoints, world_point, Vector3f, FLOAT32, true)

        /*!
         * @brief Transforms the World Points from the Raw Points using a continuous trajectory
         *
         * @note Requires timestamps field to be defined
         *       If the world points field is not defined, it will be allocated to the point cloud
         */
        void RawPointsToWorldPoints(const slam::LinearContinuousTrajectory &trajectory);

        /*!
         * @brief Transforms the World Points from Raw Points by linearly interpolating between two poses
         *
         * @note Requires timestamps field to be defined.
         *       If the world points field is not defined, it will be allocated to the point cloud
         */
        void RawPointsToWorldPoints(const slam::Pose &begin_pose, const slam::Pose &end_pose);

        /*!
         * @brief Transforms the World Points from the Raw Points using a rigid transform
         *
         * @note If the world points field is not defined, it will be allocated to the point cloud
         */
        void RawPointsToWorldPoints(const slam::SE3 &pose);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Schema Infos & Management
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Returns a const reference to the underlying buffer collection
        const BufferCollection &GetCollection() const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// View API: Get properties and elements as views
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Returns a view of a given element in the point cloud
        // Note:    The type `DestT` must have the same size as the element
        template<typename DestT>
        View<DestT> ElementView(const std::string &element_name);

        // Returns a view of a given element in the point cloud
        // Note:    The type `DestT` must have the same size as the element
        template<typename DestT>
        const View<DestT> ElementView(const std::string &element_name) const;

        // Returns a view of a property in the point cloud
        template<typename DestT>
        View<DestT> PropertyView(const std::string &element_name, const std::string &property_name);

        // Returns a view of a property in the point cloud
        template<typename DestT>
        const View<DestT> PropertyView(const std::string &element_name, const std::string &property_name) const;

        // Returns a Proxy View of a property in the point cloud
        // Note:    It is only compatible with an element if all its properties have the same type
        template<typename DestT>
        ProxyView<DestT> ElementProxyView(const std::string &element_name);

        // Returns a Proxy View of a property in the point cloud
        // Note:    It is only compatible with an element if all its properties have the same type
        template<typename DestT>
        const ProxyView<DestT> ElementProxyView(const std::string &element_name) const;

        template<typename DestT>
        ProxyView<DestT> PropertyProxyView(const std::string &element_name, const std::string &property_name);

        template<typename DestT>
        const ProxyView<DestT>
        PropertyProxyView(const std::string &element_name, const std::string &property_name) const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Property / Element management API
        ///
        /// Methods to manipulate / add / remove / rename elements and properties
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Removes the Element from the schema of the point cloud
        // @note: The corresponding item buffer will only be freed if its schema is empty
        void RemoveElement(const std::string &element_name);

        // Add a structured item to the point cloud (defined by its type and schema)
        // This will allocate the necessary space
        template<typename T>
        void AddItemVectorBuffer(slam::ItemSchema &&schema);

        // Add a structured item (containing multiple elements) to the point cloud
        // The point cloud will make a Deep Copy of the vector, by copying it to a vector buffer
        template<typename T, typename Alloc_ = std::allocator<T>>
        void AddItemVectorDeepCopy(const std::vector<T, Alloc_> &item_data,
                                   slam::ItemSchema &&schema);


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Selection API
        ///
        /// Methods to select subsets of the point cloud
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @returns A PointCloud managing its own memory, copying all the data for the corresponding indices
         */
        slam::PointCloudPtr SelectPoints(const std::vector<size_t> &indices) const;

        // TODO: Make a filter API to avoid copying the selected point cloud

        bool HaveSameSchema(const slam::PointCloud &other) const;

        /**
         * @brief Appends the point cloud to the current point cloud.
         * @note  Requires that the two point clouds have the same underlying schema
         */
        void AppendPointCloud(const slam::PointCloud &cloud);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Resizable PointCloud API
        ///
        /// All method will raise an exception if at least one buffer is not Resizable,
        /// In which case IsResizable() would return false.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        bool IsResizable() const;

        void resize(size_t new_size);

        void reserve(size_t new_size);

        template<typename T>
        void PushBackElement(const std::string &element_name, const T &element);;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Shallow / Deep Copy API
        ///
        /// These methods provide shallow / deep copies of the current point cloud.
        /// Shallow copies can restrict the view, providing access to less fields.
        /// Deep copies can change the layout, in which case the memory is managed
        /// By the returned PointCloud (via VectorBuffer collections)
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Makes a Deep Copy of the PointCloud (all buffers will be copied)
        PointCloud DeepCopy() const;

        // Makes a Deep Copy of the PointCloud (all buffers will be copied)
        PointCloudPtr DeepCopyPtr() const;

        /**
         * @returns A Resizable PointCloud with the same schema than the current point cloud
         */
        PointCloudPtr EmptyCopyPtr() const;

//        // Whether the point cloud manages its own memory
//        // ie every one of its ItemBuffer manages its own memory,
//        bool ManagesOwnMemory() const;
//
//        // Copy constructor makes a deep copy of the data
//        PointCloud(const PointCloud &pointcloud);
//
//        // Move constructor transfer ownership of the buffer collections resources
//        PointCloud(PointCloud &&pointcloud);
//
//        // Assignment operator builds a deep copy of the pointcloud
//        PointCloud &operator=(const PointCloud &pointcloud);
//
//        // Move Assignment operator transfer the buffer_collections resources
//        PointCloud &operator=(PointCloud &&pointcloud);
//
//        // Returns a deep copy of the point cloud, in which all the fields are contiguously
//        // Laid in memory, in a common Item
//        PointCloud TightFitDeepCopy() const;
//
//        // Returns a deep copy of the point cloud, in which all the selected elements
//        // Are laid contiguously in memory
//        PointCloud TightFitDeepCopy(const std::vector<std::string> &element_names) const;
//
//        // Returns a shallow copy of the point cloud.
//        // Its buffers will simply provides views of the source point cloud buffers.
//        // The Shallow copy is only allowed on Shared pointer
//        static std::shared_ptr<PointCloud> ShallowCopy(std::shared_ptr<PointCloud> cloud_ptr);


    private:
        BufferCollection collection_;

        // Fields (optional and required)
        Field xyz_; //< Required field
        std::map<std::string, Field> registered_fields_;
        // TODO : Make a PLY conversion using fields rather than ItemSchema

        // Dependency management
        std::shared_ptr<PointCloud> parent_pointcloud_ = nullptr;
        std::list<PointCloud *> child_clouds_;
    };



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename ScalarT>
    ProxyView<Eigen::Matrix<ScalarT, 3, 1>> PointCloud::XYZ() {
        static_assert(std::is_floating_point_v<ScalarT>);
        if (xyz_.IsElement()) {
            auto &xyz_element = collection_.GetElement(*xyz_.element_name);
            SLAM_CHECK_STREAM(xyz_element.properties.size() == 3, "Not a proper XYZ element defined in the schema");
            SLAM_CHECK_STREAM(xyz_element.properties[0].type == xyz_element.properties[1].type &&
                              xyz_element.properties[0].type == xyz_element.properties[2].type,
                              "Not a proper XYZ element defined in the schema");
        }
        if (xyz_.IsProperty()) {
            auto &property = collection_.GetElement(*xyz_.element_name).GetProperty(*xyz_.property_name);
            SLAM_CHECK_STREAM((property.type == FLOAT32 || property.type == FLOAT64) && property.dimension == 3,
                              "The property is incompatible with a vector of points!!!");
        }
        return ProxyFieldView<Eigen::Matrix<ScalarT, 3, 1>>(xyz_);
    }


    /* -------------------------------------------------------------------------------------------------------------- */

#define DEFAULT_XYZ_BUFFERS \
    static_assert(std::is_floating_point_v<ScalarT>); \
    using Vec3 = Eigen::Matrix<ScalarT, 3, 1>; \
    std::vector<ItemBufferPtr> buffers(1); \
    ItemBufferPtr ptr = std::make_unique<VectorBuffer>( \
            ItemSchema::Builder(sizeof(Vec3)) \
                    .AddElement("vertex", 0). \
                            template AddScalarProperty<ScalarT>("vertex", "X", 0). \
                            template AddScalarProperty<ScalarT>("vertex", "Y", sizeof(ScalarT)). \
                            template AddScalarProperty<ScalarT>("vertex", "Z", 2 * sizeof(ScalarT)) \
                    .Build(), \
            sizeof(Vec3)); \
    buffers[0] = std::move(ptr);

    template<typename ScalarT>
    PointCloudPtr PointCloud::DefaultXYZPtr() {
        DEFAULT_XYZ_BUFFERS
        return std::make_shared<PointCloud>(BufferCollection(std::move(buffers)), "vertex");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename ScalarT>
    PointCloud PointCloud::DefaultXYZ() {
        DEFAULT_XYZ_BUFFERS
        return PointCloud(BufferCollection(std::move(buffers)), "vertex");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void PointCloud::PushBackElement(const std::string &element_name, const T &element) {
        CHECK(IsResizable()) << "Cannot append an element to a non resizable buffer" << std::endl;
        CHECK(GetCollection().HasElement(element_name))
                        << "The element " << element_name << " does not exist in the collection";
        collection_.InsertItems(1);
        auto view = collection_.template element<T>(element_name);
        view.back() = element;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void PointCloud::AddItemVectorBuffer(ItemSchema &&schema) {
        CHECK(schema.GetItemSize() == sizeof(T)) << "Incompatible data sizes" << std::endl;
        for (auto &element: schema.GetElementNames()) {
            CHECK(!collection_.HasElement(element))
                            << "The point cloud already has an element with the name: "
                            << element << std::endl;
        }
        auto buffer_ptr = std::make_unique<slam::VectorBuffer>(std::move(schema), sizeof(T));
        buffer_ptr->Resize(collection_.NumItemsPerBuffer());
        collection_.AddBuffer(std::move(buffer_ptr));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T, typename Alloc_>
    void PointCloud::AddItemVectorDeepCopy(const std::vector<T, Alloc_> &item_data, ItemSchema &&schema) {
        CHECK(item_data.size() == size()) << "Incompatible sizes (pointcloud:"
                                          << size() << "/ new element: " << item_data.size() << ")" << std::endl;
        CHECK(schema.GetItemSize() == sizeof(T)) << "Incompatible data sizes" << std::endl;
        for (auto &element: schema.GetElementNames()) {
            CHECK(!collection_.HasElement(element))
                            << "The point cloud already has an element with the name: "
                            << element << std::endl;
        }
        auto buffer_ptr = std::make_unique<slam::VectorBuffer>(std::move(schema), sizeof(T));
        buffer_ptr->Reserve(collection_.NumItemsPerBuffer());
        buffer_ptr->InsertItems(item_data.size(), reinterpret_cast<const char *>(&item_data[0]));
        collection_.AddBuffer(std::move(buffer_ptr));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename DestT>
    View<DestT> PointCloud::ElementView(const std::string &element_name) {
        return collection_.element<DestT>(element_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename DestT>
    const View<DestT> PointCloud::ElementView(const std::string &element_name) const {
        return collection_.element<DestT>(element_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    template<typename DestT>
    View<DestT> PointCloud::PropertyView(const std::string &element_name, const std::string &property_name) {
        return collection_.property<DestT>(element_name, property_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename DestT>
    const View<DestT>
    PointCloud::PropertyView(const std::string &element_name, const std::string &property_name) const {
        return collection_.property<DestT>(element_name, property_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename DestT>
    ProxyView<DestT> PointCloud::ElementProxyView(const std::string &element_name) {
        return collection_.template element_proxy<DestT>(element_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename DestT>
    const ProxyView<DestT> PointCloud::ElementProxyView(const std::string &element_name) const {
        return collection_.template element_proxy<DestT>(element_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename DestT>
    ProxyView<DestT> PointCloud::PropertyProxyView(const std::string &element_name, const std::string &property_name) {
        return collection_.template property_proxy<DestT>(element_name, property_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename DestT>
    const ProxyView<DestT> PointCloud::PropertyProxyView(const std::string &element_name,
                                                         const std::string &property_name) const {
        return collection_.template property_proxy<DestT>(element_name, property_name);
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename ScalarT>
    const ProxyView<Eigen::Matrix<ScalarT, 3, 1>> PointCloud::XYZConst() const {
        static_assert(std::is_floating_point_v<ScalarT>);
        return ProxyFieldView<Eigen::Matrix<ScalarT, 3, 1>>(xyz_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename ItemT, typename Alloc_>
    PointCloud
    PointCloud::WrapVector(std::vector<ItemT, Alloc_> &data, ItemSchema &&schema, const std::string &xyz_element) {
        CHECK(schema.HasElement(xyz_element)) << "The schema does not contain the xyz element";
        const auto &elem_info = schema.GetElementInfo(xyz_element);
        CHECK(elem_info.properties.size() == 3 ||
              elem_info.properties.size() == 1 && elem_info.properties.front().dimension == 3);
        for (auto &property: elem_info.properties)
            CHECK(property.type == elem_info.properties.front().type);

        return PointCloud(
                slam::BufferCollection(slam::BufferWrapper::CreatePtr(data, std::move(schema))),
                std::string(xyz_element));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
#define __FIELD_VIEW_IMPL \
    if (property.IsItem()) \
    return collection_.template item<T>(property.item_index); \
    if (property.IsElement()) \
    return collection_.template element<T>(*property.element_name); \
    if (property.IsProperty()) \
    return collection_.template property<T>(*property.element_name, *property.property_name); \
    throw std::runtime_error("Unexpected Property Type");


    template<typename T>
    View<T> PointCloud::FieldView(const PointCloud::Field &property) {
        __FIELD_VIEW_IMPL
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const View<T> PointCloud::FieldView(const PointCloud::Field &property) const {
        __FIELD_VIEW_IMPL
    }

    /* -------------------------------------------------------------------------------------------------------------- */
#define __PROXY_FIELD_IMPL \
    SLAM_CHECK_STREAM(!property.IsItem(), "An Item cannot be used with a Proxy view (property: " << property.item_index << ", " \
                        << (property.element_name ? "" : *property.element_name) << " , "                                       \
                        << (property.property_name ?  "" : *property.property_name)); \
    if (property.IsElement()) \
    return collection_.template element_proxy<T>(*property.element_name); \
    if (property.IsProperty()) \
    return collection_.template property_proxy<T>(*property.element_name, *property.property_name); \
    throw std::runtime_error("Unexpected Property Type");

    template<typename T>
    ProxyView<T> PointCloud::ProxyFieldView(const PointCloud::Field &property) {
        __PROXY_FIELD_IMPL
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const ProxyView<T> PointCloud::ProxyFieldView(const PointCloud::Field &property) const {
        __PROXY_FIELD_IMPL
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename ItemT, typename Alloc_>
    const PointCloud PointCloud::WrapConstVector(const std::vector<ItemT, Alloc_> &data, ItemSchema &&schema,
                                                 const std::string &xyz_element) {
        return WrapVector(const_cast<std::vector<ItemT, Alloc_> &>(data),
                          std::move(schema),
                          xyz_element);
    }

    /* -------------------------------------------------------------------------------------------------------------- */

#define __ADD_FIELD                                                                             \
    SLAM_CHECK_STREAM(!collection_.HasElement(element_name),                                    \
    "The element `" << element_name << "` already exists in the schema")                        \
    SLAM_CHECK_STREAM(registered_fields_.find(element_name) == registered_fields_.end(),        \
                      "The element `" << element_name << "` already is registered")             \
                                                                                                \
    constexpr int pty_size = StaticPropertySize<pty_type>();                                    \
    constexpr int data_size = sizeof(DataT);                                                    \
    static_assert(data_size >= pty_size && ((data_size / pty_size) * pty_size == data_size),    \
                  "The DataType cannot be reinterpreted as vector of the basic property type"); \
    const auto dim = data_size / pty_size;                                                      \
    ItemSchema::Builder builder(data_size);                                                     \
    builder.AddElement(element_name, 0);                                                        \
    builder.AddProperty(element_name, std::string(element_name), pty_type, 0, dim);             \
    AddItemVectorBuffer<DataT>(builder.Build());                                                \
    Field field{                                                                                \
            int(collection_.GetItemIndex(element_name)),                                        \
            {element_name},                                                                     \
            {}                                                                                  \
    };                                                                                          \
    registered_fields_[element_name] = field;

    template<typename DataT, slam::PROPERTY_TYPE pty_type>
    PointCloud::Field PointCloud::AddElementField(const std::string &element_name, const std::vector<DataT> &data) {
        SLAM_CHECK_STREAM(size() == data.size(),
                          "Incompatible sizes !")
        __ADD_FIELD
        auto data_view = FieldView<DataT>(field);
        std::copy(data.begin(), data.end(), data_view.begin());
        return field;
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename DataT, slam::PROPERTY_TYPE pty_type>
    PointCloud::Field PointCloud::AddElementField(const std::string &element_name) {
        __ADD_FIELD
        return field;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename PointT, typename ScalarT>
    std::enable_if_t<std::is_same_v<decltype(PointT::xyz), Eigen::Matrix<ScalarT, 3, 1>>,
            slam::PointCloudPtr> PointCloud::MakeEmptyPointCloud(std::optional<slam::ItemSchema> schema,
                                                                 const std::string &xyz_element) {
        slam::ItemSchema::Builder builder(sizeof(PointT));
        bool add_xyz_element = true;
        if (schema && schema->HasElement(xyz_element)) {
            add_xyz_element = false;
            builder = schema->GetBuilder();
        }
        if (add_xyz_element) {
            // Add an element to the schema
            auto offset_of_x = offsetof(PointT, xyz);
            builder.AddElement(xyz_element, offsetof(PointT, xyz));
            builder.AddProperty(xyz_element, "x", slam::StaticPropertyType<ScalarT>(), offset_of_x, 1);
            builder.AddProperty(xyz_element, "y", slam::StaticPropertyType<ScalarT>(),
                                offset_of_x + sizeof(ScalarT), 1);
            builder.AddProperty(xyz_element, "z", slam::StaticPropertyType<ScalarT>(),
                                offset_of_x + 2 * sizeof(ScalarT),
                                1);
        }
        auto item_buffer = std::make_unique<slam::VectorBuffer>(builder.Build(), sizeof(PointT));
        auto buffer_collection = slam::BufferCollection(std::move(item_buffer));
        return std::make_shared<slam::PointCloud>(std::move(buffer_collection), std::string(xyz_element));
    }


} // namespace slam


#endif //SlamCore_POINTCLOUD_H
