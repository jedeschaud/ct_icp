#include "SlamCore/data/buffer_collection.h"

namespace slam {


    /* -------------------------------------------------------------------------------------------------------------- */
    const ElementInfo &BufferCollection::GetElement(const std::string &element) const {
        for (auto &item_buffer: item_buffers) {
            if (item_buffer) {
                if (item_buffer->item_info.HasElement(element))
                    return item_buffer->item_info.item_schema.GetElementInfo(element);
            }
        }
        throw std::runtime_error("[BufferCollection][ElementInfo]Element " + element + " not found");
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool BufferCollection::HasElement(const std::string &name) const {
        return std::find_if(item_buffers.begin(),
                            item_buffers.end(), [&name](const ItemBufferPtr &ptr) {
                    return ptr && (*ptr).item_info.HasElement(name);
                }) != item_buffers.end();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool BufferCollection::HasProperty(const std::string &elem_name, const std::string &property) const {
        if (!HasElement(elem_name))
            return false;
        return GetElement(elem_name).HasProperty(property);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool BufferCollection::IsResizable() const {
        return std::all_of(item_buffers.begin(), item_buffers.end(), [&](const ItemBufferPtr &ptr) {
            return ptr->IsResizable();
        });
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemInfo &BufferCollection::GetItemInfo(size_t item_index) {
        CHECK(item_index < item_buffers.size()) << "Invalid item index" << std::endl;
        return item_buffers[item_index]->item_info;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t BufferCollection::NumItemsPerBuffer() const {
        CHECK(!item_buffers.empty()) << "Empty collection" << std::endl;
        return item_buffers[0]->NumItems();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    BufferCollection::BufferCollection(std::vector<ItemBufferPtr> &&buffer_ptr) {
        if (!buffer_ptr.empty()) {
            CHECK(AreSizesConsistent(buffer_ptr)) << "Sizes are consistent" << std::endl;
        }
        item_buffers = std::move(buffer_ptr);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    BufferCollection::BufferCollection(ItemBufferPtr &&buffer_ptr) {
        item_buffers.resize(1);
        item_buffers[0] = std::move(buffer_ptr);
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    bool BufferCollection::AreSizesConsistent(const std::vector<ItemBufferPtr> &buffer) {
        if (buffer.empty())
            return false;
        if (!buffer[0])
            return false;
        size_t num_items = buffer[0]->NumItems();
        for (auto i(1); i < buffer.size(); ++i)
            if (num_items != buffer[0]->NumItems())
                return false;
        return true;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void BufferCollection::AddBuffer(ItemBufferPtr &&buffer_ptr) {
        CHECK(buffer_ptr) << "Cannot add empty buffer pointer" << std::endl;
        CHECK(buffer_ptr->NumItems() == NumItemsPerBuffer()) << "Invalid number of items" << std::endl;
        for (auto element_name: buffer_ptr->item_info.item_schema.GetElementNames()) {
            CHECK(!HasElement(element_name)) << "An element with name " << element_name <<
                                             "already exists in the buffer" << std::endl;
        }
        item_buffers.emplace_back(std::move(buffer_ptr));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    int BufferCollection::NumItemsInSchema() const {
        return static_cast<int>(std::accumulate(item_buffers.begin(),
                                                item_buffers.end(),
                                                0,
                                                [](int acc, const ItemBufferPtr &buffer_ptr) {
                                                    return acc + (buffer_ptr ? 1 : 0);
                                                }));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void BufferCollection::Resize(size_t new_size) {
        for (auto &buffer: item_buffers) {
            CHECK(buffer->IsResizable());
            dynamic_cast<ResizableBuffer &>(*buffer).Resize(new_size);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void BufferCollection::Reserve(size_t new_size) {
        for (auto &buffer: item_buffers) {
            CHECK(buffer->IsResizable());
            dynamic_cast<ResizableBuffer &>(*buffer).Reserve(new_size);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void BufferCollection::RemoveElement(const std::string &element_name, bool remove_if_empty) {
        if (HasElement(element_name)) {
            size_t item_index = GetItemIndex(element_name);
            auto &item = item_buffers[item_index];
            item->RemoveElementFromSchema(element_name);

            // No more elements in the item, remove it from the collection
            if (remove_if_empty && item->GetItemSchema().GetTotalElementSize() == 0) {
                item.reset();
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t BufferCollection::GetItemIndex(const std::string &element_name) const {
        size_t idx(0);
        for (auto &buffer: item_buffers) {
            if (buffer && buffer->item_info.HasElement(element_name))
                return idx;
            idx++;
        }
        throw std::runtime_error("The collection has no element with name " + element_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    const ItemInfo &BufferCollection::GetItemInfo(size_t item_index) const {
        CHECK(item_index < item_buffers.size()) << "Bad Index" << std::endl;
        CHECK(item_buffers[item_index]) << "The item buffer at index " << item_index << std::endl;
        return item_buffers[item_index]->item_info;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void BufferCollection::InsertItems(size_t num_items) {
        CHECK(IsResizable());
        for (auto &buffer_ptr: item_buffers) {
            dynamic_cast<ResizableBuffer &>(*buffer_ptr).InsertItems(num_items);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool
    BufferCollection::HasMatchingProperty(const std::string &elem_name, const std::string &property, PROPERTY_TYPE type,
                                          int dim) const {
        if (!HasProperty(elem_name, property))
            return false;
        auto &pinfo = GetElement(elem_name).GetProperty(property);
        return pinfo.type == type && pinfo.dimension == dim;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    BufferCollection BufferCollection::DeepCopy() const {
        std::vector<slam::ItemBufferPtr> buffers;
        for (auto &buffer: item_buffers) {
            auto buffer_ptr = VectorBuffer::DeepCopyPtr(*buffer);
            buffers.emplace_back(std::move(buffer_ptr));
        }
        return BufferCollection(std::move(buffers));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    BufferCollection BufferCollection::EmptyCopy() const {
        std::vector<slam::ItemBufferPtr> buffers;
        for (auto &buffer: item_buffers) {
            auto buffer_ptr = std::make_unique<VectorBuffer>(buffer->GetItemSchema().GetBuilder().Build(),
                                                             buffer->item_info.item_size);
            buffers.emplace_back(std::move(buffer_ptr));
        }
        return BufferCollection(std::move(buffers));
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    BufferCollection BufferCollection::SelectItems(const std::vector<size_t> &indices) const {
        auto collection = EmptyCopy();
        const auto kNumSelectedPoints = indices.size();
        collection.Resize(kNumSelectedPoints);

        int num_buffers = NumItemsInSchema();
        for (auto buffer_idx(0); buffer_idx < num_buffers; buffer_idx++) {
            auto &buffer = item_buffers[buffer_idx];
            auto &dest_buffer = collection.item_buffers[buffer_idx];

            const char *old_buffer_ptr = buffer->view_data_ptr;
            char *buffer_ptr = dest_buffer->view_data_ptr;
            const auto kItemSize = buffer->item_info.item_size;
            for (auto item_idx(0); item_idx < kNumSelectedPoints; ++item_idx) {
                auto old_item_idx = indices[item_idx];
                SLAM_CHECK_STREAM(old_item_idx < NumItemsPerBuffer(),
                                  "Invalid index! " << old_item_idx << " Not in the range[0, "
                                                    << kNumSelectedPoints << "]");

                // Copy Item Data
                std::copy(old_buffer_ptr + old_item_idx * kItemSize,
                          old_buffer_ptr + (old_item_idx + 1) * kItemSize,
                          buffer_ptr + item_idx * kItemSize);

            }
        }
        return collection;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void BufferCollection::Append(const BufferCollection &collection) {
        SLAM_CHECK_STREAM(IsResizable(), "Cannot append points to a non-resizable point cloud");
        SLAM_CHECK_STREAM(HasSameSchema(collection), "The Two collections do not have the same schema");
        size_t old_size = NumItemsPerBuffer();
        size_t num_items = collection.NumItemsPerBuffer();
        Resize(old_size + num_items);

        for (auto idx(0); idx < NumItemsInSchema(); ++idx) {
            auto &item_info = GetItemInfo(idx);
            auto &other_item_info = collection.GetItemInfo(idx);

            auto dest_ptr = (item_info.parent_buffer->view_data_ptr + item_info.item_size * old_size);
            auto src_ptr = other_item_info.parent_buffer->view_data_ptr;
            std::copy(src_ptr, src_ptr + num_items * other_item_info.item_size, dest_ptr);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool BufferCollection::HasSameSchema(const BufferCollection &other) const {
        if (other.NumItemsInSchema() != NumItemsInSchema())
            return false;
        for (auto idx(0); idx < other.NumItemsInSchema(); idx++) {
            auto &item_info = GetItemInfo(idx);
            auto &other_item_info = other.GetItemInfo(idx);
            if (item_info.item_size != other_item_info.item_size)
                return false;
        }
        return true;
    }

    /* -------------------------------------------------------------------------------------------------------------- */

}