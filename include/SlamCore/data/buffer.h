#ifndef SlamCore_BUFFER_H
#define SlamCore_BUFFER_H

#include <numeric>
#include <memory>
#include <map>

#include <Eigen/Dense>
#include <glog/logging.h>

#include "SlamCore/utils.h"
#include "SlamCore/data/schema.h"

namespace slam {

    /*!
     * @brief   An ItemBuffer provides access to Items data, which are continuously laid out in memory.
     *          Providing all the necessary information to decode their byte array into items, elements and properties.
     */
    struct ItemBuffer {

        virtual ~ItemBuffer() = 0;

        ItemBuffer(ItemSchema &&schema, int item_size) : item_info(std::move(schema), *this, item_size) {};

        virtual size_t NumItems() const = 0;

        // Whether the Items Buffer is resizable
        virtual bool IsResizable() const { return false; }

        // Removes an Item from the schema
        // Note, this does not actually change the layout of the data
        void RemoveElementFromSchema(const std::string &element_name);

        // Returns a const reference to the item schema
        const ItemSchema &GetItemSchema() const;

        template<typename ItemT>
        void CheckValidIndex(size_t index) const {
            CHECK(index < NumItems()) << "Item out of bound" << std::endl;
            CHECK(sizeof(ItemT) == item_info.item_size)
                            << "The size of the template parameter type does not match the layout" << std::endl;
            CHECK(view_data_ptr) << "The Data points to a nullptr. Invalid ItemBuffer." << std::endl;
        }

        // Returns a reference to an item in the buffer
        template<typename ItemT>
        ItemT &At(size_t index);

        // Returns a const reference to an item in the buffer
        template<typename ItemT>
        const ItemT &At(size_t index) const {
            CheckValidIndex<ItemT>(index);
            const ItemT *item_ptr = reinterpret_cast<const ItemT *>(view_data_ptr);
            return item_ptr[index];
        }

        char *view_data_ptr = nullptr;  // A char Pointer to the data (to avoid virtual calls) managed by child classes

        ItemInfo item_info;             // The schema of the data layout for the items in the buffer

    };


    /*!
     * @brief   A ResizableBuffer provides an interface for resizable buffer managing their own memory.
     *          It provides a vector like interface for managing the memory of its items.
     */
    struct ResizableBuffer : ItemBuffer {

        using ItemBuffer::NumItems;
        using ItemBuffer::ItemBuffer;

        virtual ~ResizableBuffer() = 0;

        // Returns the maximum number of items which can be inserted into the buffer
        virtual size_t MaxNumItems() const { return std::numeric_limits<size_t>::max() / item_info.item_size; }

        // Resizes the buffer to `num_items`
        virtual void Resize(size_t num_items) = 0;

        // Reserves `num_items` into the buffer
        virtual void Reserve(size_t num_items) = 0;

        // Reserves an additional `num_items` into the buffer
        virtual void ReserveAdditional(size_t num_items) = 0;

        // Inserts num_items into the buffer
        virtual void InsertItems(size_t num_items, const char *items_data = nullptr) = 0;

        // Templated insert into the buffer
        template<typename T>
        void PushBackItem(const T &item);

        // Returns the pointer to the first byte of the item at idx `item_idx`
        inline char *AtPtr(size_t item_idx) { return view_data_ptr + (item_idx * item_info.item_size); };

        // Returns the pointer to the first byte of the last item
        inline char *BackPtr() {
            return std::max(view_data_ptr + (NumItems() - 1) * item_info.item_size, view_data_ptr);
        }

        inline size_t Size() const { return NumItems(); }

        bool IsResizable() const final { return true; }
    };


    /*!
     * @brief   A VectorBuffer if a Resizable buffer which stores its data on a heap allocated vector of bytes
     */
    class VectorBuffer : public ResizableBuffer {
    private:
        std::vector<char> data;
    public:
        ~VectorBuffer() override;

        explicit VectorBuffer(ItemSchema &&schema, int item_size)
                : ResizableBuffer(std::move(schema), item_size) {}

        template<typename T, typename Alloc_ = std::allocator<T>>
        static VectorBuffer Copy(std::vector<T, Alloc_> &items,
                                 ItemSchema &&info);

        size_t NumItems() const override;

        void InsertItems(size_t num_items, const char *items_data = nullptr) override;

        // Resizes the buffer to `num_items`
        void Resize(size_t num_items) override;

        // Reserves `num_items` into the buffer
        void Reserve(size_t num_items) override;

        // Reserves an additional `num_items` into the buffer
        void ReserveAdditional(size_t num_items) override;

        // Returns a Deep Copy of a given ItemBuffer
        VectorBuffer static DeepCopy(const ItemBuffer &other);

        // Returns a Deep Copy of a given ItemBuffer
        std::unique_ptr<VectorBuffer> static DeepCopyPtr(const ItemBuffer &other);

    };

    /*!
     * @brief   An ArrayBuffer if a Resizable buffer which stores its data contiguously,
     *          With a buffer size known at compile time
     */
    template<size_t MaxSize>
    class ArrayBuffer : public ResizableBuffer {
    private:
        std::array<char, MaxSize> data_;
        size_t allocated_ = 0;
        const size_t capacity_;
    public:
        ~ArrayBuffer() override;

        size_t MaxCapacity() const;

        explicit ArrayBuffer(ItemSchema &&schema, int item_size)
                : ResizableBuffer(std::move(schema), item_size), capacity_(MaxSize / item_size) {
            view_data_ptr = data_.data();
        }

        template<typename T, typename Alloc_ = std::allocator<T>>
        static ArrayBuffer Copy(std::vector<T, Alloc_> &items,
                                ItemSchema &&info);

        size_t NumItems() const override;

        void InsertItems(size_t num_items, const char *items_data = nullptr) override;;

        // Resizes the buffer to `num_items`
        void Resize(size_t num_items) override;;

        // Reserves `num_items` into the buffer
        void Reserve(size_t num_items) override;;

        // Reserves an additional `num_items` into the buffer
        void ReserveAdditional(size_t num_items) override;;

    };


    /*!
     * @brief   A BufferWrapper if a non-resizable buffer which offers a view of an already allocated buffer.
     *
     * The BufferWrapper contains an optional logic to keep track of the base data via a smart pointer,
     * To simplify the memory management.
     */
    class BufferWrapper : public ItemBuffer {
    public:

        // An abstract wrapper to a smart pointer, to facilitate the memory flow
        struct SmartDataPtrWrapper {
            virtual ~SmartDataPtrWrapper() = 0;
        };

        // A templated Wrapper to a shared_ptr
        template<typename BaseDataType>
        struct SharedPtrDataPtr : SmartDataPtrWrapper {

            ~SharedPtrDataPtr() override = default;

            std::shared_ptr<BaseDataType> data_ptr = nullptr;
        };

    private:
        size_t num_items_;
        std::shared_ptr<SmartDataPtrWrapper> smart_data_ptr_ = nullptr;  // An optional smart data pointer wrapper (to prevent spurious data deletions)
    public:

        explicit BufferWrapper(ItemSchema &&view,
                               char *buffer,
                               size_t num_items,
                               int item_size,
                               std::shared_ptr<SmartDataPtrWrapper> smart_ptr = nullptr);

        template<typename T, typename Alloc_ = std::allocator<T>>
        static BufferWrapper Create(std::vector<T, Alloc_> &items,
                                    ItemSchema &&info);

        template<typename T, typename Alloc_ = std::allocator<T>>
        static BufferWrapper Create(std::shared_ptr<std::vector<T, Alloc_>> items_ptr,
                                    ItemSchema &&info);

        template<typename T, typename Alloc_ = std::allocator<T>>
        static std::unique_ptr<BufferWrapper> CreatePtr(std::vector<T, Alloc_> &items,
                                                        ItemSchema &&info);

        template<typename T, typename Alloc_ = std::allocator<T>>
        static std::unique_ptr<BufferWrapper> CreatePtr(std::shared_ptr<std::vector<T, Alloc_>> items_ptr,
                                                        ItemSchema &&info);

        bool IsResizable() const override { return false; }

        size_t NumItems() const override { return num_items_; }
    };


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename ItemT>
    ItemT &ItemBuffer::At(size_t index) {
        CheckValidIndex<ItemT>(index);
        ItemT *item_ptr = reinterpret_cast<ItemT *>(view_data_ptr);
        return item_ptr[index];
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void ResizableBuffer::PushBackItem(const T &item) {
        CHECK(sizeof(item) == item_info.item_size)
                        << "The item size does not match the template param size" << std::endl;
        const char *item_data = reinterpret_cast<const char *>(&item);
        InsertItems(1, item_data);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T, typename Alloc_>
    BufferWrapper BufferWrapper::Create(std::vector<T, Alloc_> &items, ItemSchema &&info) {
        return BufferWrapper(std::move(info),
                             reinterpret_cast<char *>(&items[0]),
                             items.size(),
                             sizeof(T));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T, typename Alloc_>
    BufferWrapper BufferWrapper::Create(std::shared_ptr<std::vector<T, Alloc_>> items_ptr,
                                        ItemSchema &&info) {
        CHECK(items_ptr) << "Null pointer cannot be used as reference to a Buffer Wrapper" << std::endl;
        return BufferWrapper(
                std::move(info),
                reinterpret_cast<char *>(&items_ptr->at(0)),
                items_ptr->size(),
                sizeof(T));
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T, typename Alloc_>
    std::unique_ptr<BufferWrapper> BufferWrapper::CreatePtr(std::shared_ptr<std::vector<T, Alloc_>> items_ptr,
                                                            ItemSchema &&info) {
        CHECK(items_ptr) << "Null pointer cannot be used as reference to a Buffer Wrapper" << std::endl;
        return std::make_unique<BufferWrapper>(std::move(info),
                                               reinterpret_cast<char *>(&items_ptr->at(0)),
                                               items_ptr->size(),
                                               sizeof(T));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T, typename Alloc_>
    std::unique_ptr<BufferWrapper> BufferWrapper::CreatePtr(std::vector<T, Alloc_> &items,
                                                            ItemSchema &&info) {
        return std::make_unique<BufferWrapper>(std::move(info),
                                               reinterpret_cast<char *>(&items[0]),
                                               items.size(),
                                               sizeof(T));
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T, typename Alloc_>
    VectorBuffer VectorBuffer::Copy(std::vector<T, Alloc_> &items, ItemSchema &&info) {
        CHECK(info.GetItemSize() == sizeof(T)) << "Incompatible item size" << std::endl;
        VectorBuffer buffer(std::move(info), sizeof(T));
        buffer.Reserve(items.size());
        for (auto i(0); i < items.size(); ++i)
            buffer.InsertItems(1, reinterpret_cast<const char *>(&items[i]));
        return buffer;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<size_t MaxSize>
    ArrayBuffer<MaxSize>::~ArrayBuffer() {}

    /* -------------------------------------------------------------------------------------------------------------- */
    template<size_t MaxSize>
    size_t ArrayBuffer<MaxSize>::MaxCapacity() const { return capacity_; }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<size_t MaxSize>
    template<typename T, typename Alloc_>
    ArrayBuffer<MaxSize> ArrayBuffer<MaxSize>::Copy(std::vector<T, Alloc_> &items, ItemSchema &&info) {
        auto item_size = info.GetItemSize();
        CHECK(sizeof(T) == item_size);
        ArrayBuffer buffer(std::move(info), item_size);
        CHECK(items.size() <= buffer.MaxCapacity());
        buffer.Reserve(items.size());
        for (auto &item: items) {
            buffer.template PushBackItem<T>(item);
        }
        return buffer;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<size_t MaxSize>
    size_t ArrayBuffer<MaxSize>::NumItems() const {
        return allocated_;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<size_t MaxSize>
    void ArrayBuffer<MaxSize>::InsertItems(size_t num_items, const char *items_data) {
        SLAM_CHECK_STREAM(allocated_ + num_items <= capacity_, "");
        const auto item_size = item_info.item_size;
        if (items_data)
            std::copy(items_data, items_data + num_items * item_size, data_.data() + allocated_ * item_size);
        allocated_ += num_items;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<size_t MaxSize>
    void ArrayBuffer<MaxSize>::Resize(size_t num_items) {
        SLAM_CHECK_STREAM(num_items <= capacity_, "The new size `num_items` is larger than the array's capacity");
        allocated_ = num_items;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<size_t MaxSize>
    void ArrayBuffer<MaxSize>::Reserve(size_t num_items) {
        if (num_items > capacity_)
            LOG(WARNING) << "[ArrayBuffer] Reserve was called with a value larger than the number of items";
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<size_t MaxSize>
    void ArrayBuffer<MaxSize>::ReserveAdditional(size_t num_items) {
        if (num_items + allocated_ > capacity_)
            LOG(WARNING) << "[ArrayBuffer] Reserve was called with a value larger than the number of items";
    }


} // namespace slam


#endif //SlamCore_BUFFER_H
