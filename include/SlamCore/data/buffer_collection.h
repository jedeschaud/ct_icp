#ifndef SlamCore_BUFFER_COLLECTION_H
#define SlamCore_BUFFER_COLLECTION_H

#include "SlamCore/data/buffer.h"
#include "SlamCore/data/view.h"

namespace slam {

    typedef std::unique_ptr<ItemBuffer> ItemBufferPtr;

    /*!
     * @brief A BufferCollection wraps a vector of ItemBuffer pointers,
     *
     * It provides helping functions for their manipulation, without modifying the data's topology
     */
    class BufferCollection {
    public:
        explicit BufferCollection(std::vector<ItemBufferPtr> &&buffer_ptr);

        explicit BufferCollection(ItemBufferPtr &&buffer_ptr);

        // A static Factory to build a collection from an arbitrary number of item buffers rvalue references
        template<typename BufferT, typename ...Args>
        static BufferCollection Factory(std::unique_ptr<BufferT> &&ptr,
                                        std::unique_ptr<Args> &&... args);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Returns views of the elements and properties in the buffer
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        template<typename T>
        View<T> item(size_t item_index);

        template<typename T>
        const View<T> item(size_t item_index) const;

        template<typename T>
        View<T> element(const std::string &element_name);

        template<typename T>
        const View<T> element(const std::string &element_name) const;

        template<typename T>
        ProxyView<T> element_proxy(const std::string &element_name);

        template<typename T>
        const ProxyView<T> element_proxy(const std::string &element_name) const;

        template<typename T>
        View<T> property(const std::string &element_name, const std::string &property_name);

        template<typename T>
        const View<T> property(const std::string &element_name, const std::string &property_name) const;

        template<typename T>
        ProxyView<T> property_proxy(const std::string &element_name, const std::string &property_name, int dim = 0);

        template<typename T>
        const ProxyView<T>
        property_proxy(const std::string &element_name, const std::string &property_name, int dim = 0) const;


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Information about the buffer and the schema
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // The number of items in each buffer
        size_t NumItemsPerBuffer() const;

        // The number of different items in the schema of the collection
        int NumItemsInSchema() const;

        // Returns whether the buffer has the given element
        bool HasElement(const std::string &name) const;

        // Returns whether the buffer has the given property
        bool HasProperty(const std::string &elem_name, const std::string &property) const;

        // Whether the property matches the type and dimension specified
        bool HasMatchingProperty(const std::string &elem_name, const std::string &property,
                                 PROPERTY_TYPE type, int dim = 1) const;

        // Returns the index of associated to the element's name
        size_t GetItemIndex(const std::string &element_name) const;;

        // Returns the ItemInfo associated to the index
        ItemInfo &GetItemInfo(size_t item_index);

        // Returns the ItemInfo associated to the index
        const ItemInfo &GetItemInfo(size_t item_index) const;;

        // Returns element
        const ElementInfo &GetElement(const std::string &element) const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Buffer Management
        ///
        /// Methods to Add / Remove Items buffers and change the schema
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Removes an Element from the buffer collection
        // If `remove_empty_item` is true, the pointers to item buffers are freed if the schemas are empty
        void RemoveElement(const std::string &element_name, bool remove_empty_item = false);

        // Adds a buffer to the collection (requires only that the number of Items is consistent)
        void AddBuffer(ItemBufferPtr &&buffer_ptr);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Resizable Buffer Collection Methods
        ///
        /// Only applicable when the IsResizable() returns true
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Whether all buffers are Resizable
        bool IsResizable() const;

        // Resizes all buffers
        void Resize(size_t new_size);

        // Reserves the size for all buffers
        void Reserve(size_t new_size);

        // Inserts `num_items` with default data (zeros characters)
        void InsertItems(size_t num_items);

        // Appends the content of another collection to the collection
        void Append(const slam::BufferCollection &collection);

        // Appends the content of another collection to the collection
        bool HasSameSchema(const slam::BufferCollection &other) const;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Copies
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Makes a deep copy of the BufferCollection, ie all the buffers in the collection are copied into VectorBuffers
        BufferCollection DeepCopy() const;

        /**
         * @returns An empty copy of the BufferCollection, replacing the item buffers by empty VectorBuffers,
         *          But keeping the same schema
         */
        BufferCollection EmptyCopy() const;

        /**
         * @returns A buffer collection consisting of VectorBuffers, containing copies of the selected point indices
         */
        BufferCollection SelectItems(const std::vector<size_t> &indices) const;

    private:
        std::vector<ItemBufferPtr> item_buffers;

        // Returns whether the sizes are consistent
        static bool AreSizesConsistent(const std::vector<ItemBufferPtr> &buffer);

        template<typename BufferT, typename ...Args>
        static void Factory_(std::unique_ptr<BufferT> &&ptr,
                             std::unique_ptr<Args> &&... args,
                             std::vector<ItemBufferPtr> &pointers) {
            ItemBufferPtr typed_ptr = std::move(ptr);
            pointers.emplace_back(std::move(typed_ptr));
            Factory_(std::move(args)..., pointers);
        };

        static void Factory_(std::vector<ItemBufferPtr> &pointers) {
            return;
        }
    };


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATION
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename BufferT, typename... Args>
    BufferCollection BufferCollection::Factory(std::unique_ptr<BufferT> &&ptr, std::unique_ptr<Args> &&... args) {
        std::vector<ItemBufferPtr> buffers;
        Factory_<BufferT, Args...>(std::move(ptr), std::move(args)..., buffers);
        CHECK(BufferCollection::AreSizesConsistent(buffers)) << "Inconsistent sizes in the buffer";
        return BufferCollection(std::move(buffers));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
#define BUFFER_COLLECTION_ELEMENT_FUNCTION \
    const auto &element_info = GetElement(element_name); \
    auto &item_info = *element_info.parent; \
    CHECK(item_info.parent_buffer) << "Invalid ItemInfo: Not attached to an ItemBuffer" << std::endl; \
    CHECK(element_info.ElementSize() == sizeof(T)) << "Invalid size: the templated parameter's size (" \
    << sizeof(T) << " does not match the buffer's element size: " \
    << element_info.ElementSize() << std::endl; \
    return View<T>(*item_info.parent_buffer,  \
    element_info.offset_in_item, \
    item_info.item_size);

    template<typename T>
    View<T> BufferCollection::element(const std::string &element_name) {
        BUFFER_COLLECTION_ELEMENT_FUNCTION
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const View<T> BufferCollection::element(const std::string &element_name) const {
        BUFFER_COLLECTION_ELEMENT_FUNCTION
    }


    /* -------------------------------------------------------------------------------------------------------------- */
#define BUFFER_COLLECTION_PROPERTY_METHOD \
    const auto &element_info = GetElement(element_name); \
    const auto &property = element_info.GetProperty(property_name); \
    const auto &item_info = *element_info.parent; \
    CHECK(item_info.parent_buffer) << "Invalid ItemInfo: Not attached to an ItemBuffer" << std::endl; \
    return View<T>(*item_info.parent_buffer, \
    element_info.offset_in_item + property.offset_in_elem, \
    item_info.item_size);

    template<typename T>
    View<T> BufferCollection::property(const std::string &element_name, const std::string &property_name) {
        BUFFER_COLLECTION_PROPERTY_METHOD
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const View<T> BufferCollection::property(const std::string &element_name, const std::string &property_name) const {
        BUFFER_COLLECTION_PROPERTY_METHOD
    }

    /* -------------------------------------------------------------------------------------------------------------- */
#define BUFFER_COLLECTION_ITEM_METHOD \
    CHECK(item_buffers.size() > index); \
    CHECK(item_buffers[index]); \
    auto &buffer = *item_buffers[index]; \
    return View<T>(buffer, 0, buffer.item_info.item_size);

    template<typename T>
    View<T> BufferCollection::item(size_t index) {
        BUFFER_COLLECTION_ITEM_METHOD
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const View<T> BufferCollection::item(size_t index) const {
        BUFFER_COLLECTION_ITEM_METHOD
    }

    /* -------------------------------------------------------------------------------------------------------------- */
#define BUFFER_COLLECTION_ELEMENT_PROXY_METHOD \
    auto &element_info = GetElement(element_name); \
    for (auto &pty: element_info.properties) \
    CHECK(pty.type == element_info.properties.front().type) << "Inconsistent property types" << std::endl; \
    return ProxyView<T>(element_info.properties.front().type, \
                        *element_info.parent->parent_buffer, \
                        element_info.offset_in_item, \
                        element_info.parent->item_size);

    template<typename T>
    ProxyView<T> BufferCollection::element_proxy(const std::string &element_name) {
        BUFFER_COLLECTION_ELEMENT_PROXY_METHOD
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const ProxyView<T> BufferCollection::element_proxy(const std::string &element_name) const {
        BUFFER_COLLECTION_ELEMENT_PROXY_METHOD
    }

    /* -------------------------------------------------------------------------------------------------------------- */
#define BUFFER_COLLECTION_PROPERTY_PROXY_METHOD \
    auto &element_info = GetElement(element_name); \
    auto &pty = element_info.GetProperty(property_name); \
    CHECK(0 <= property_dim && property_dim < pty.dimension) << "The property has " \
    << pty.dimension << " dimensions, incompatible with a property " << property_dim; \
    return ProxyView<T>(pty.type, \
                        *element_info.parent->parent_buffer, \
                        element_info.offset_in_item + pty.offset_in_elem + property_dim * PropertySize(pty.type), \
                        element_info.parent->item_size);

    template<typename T>
    ProxyView<T> BufferCollection::property_proxy(const std::string &element_name,
                                                  const std::string &property_name,
                                                  int property_dim) {
        BUFFER_COLLECTION_PROPERTY_PROXY_METHOD
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const ProxyView<T> BufferCollection::property_proxy(const std::string &element_name,
                                                        const std::string &property_name,
                                                        int property_dim) const {
        BUFFER_COLLECTION_PROPERTY_PROXY_METHOD
    }


} // namespace slam

#endif //SlamCore_BUFFER_COLLECTION_H
