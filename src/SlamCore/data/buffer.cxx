#include <glog/logging.h>
#include "SlamCore/data/buffer.h"
#include "SlamCore/data/schema.h"


namespace slam {


    namespace {
        auto accumulate_element_size = [](const std::vector<ElementInfo> element_info) {
            return std::accumulate(element_info.begin(),
                                   element_info.end(), 0,
                                   [](int acc, const ElementInfo &info) {
                                       return acc + info.ElementSize();
                                   });
        };

        auto move_and_ref = [](std::vector<ElementInfo> &&elem_info,
                               ItemInfo &parent) -> std::vector<ElementInfo> && {
            for (auto &elem: elem_info)
                elem.parent = &parent;
            return std::move(elem_info);
        };
    };

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemBuffer::~ItemBuffer() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    const ItemSchema &ItemBuffer::GetItemSchema() const { return item_info.item_schema; }

    /* -------------------------------------------------------------------------------------------------------------- */
    void ItemBuffer::RemoveElementFromSchema(const std::string &element_name) {
        if (item_info.HasElement(element_name)) {
            auto builder = item_info.item_schema.GetBuilder().RemoveElement(element_name);
            item_info = ItemInfo(builder.Build(), *this, builder.GetItemSize());
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ResizableBuffer::~ResizableBuffer() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    size_t VectorBuffer::NumItems() const {
        return data.size() / item_info.item_size;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void VectorBuffer::InsertItems(size_t num_items, const char *items_data) {
        const int item_size = item_info.item_size;
        const size_t new_items_size = num_items * item_size;
        for (auto i(0); i < new_items_size; ++i)
            data.push_back(items_data ? items_data[i] : char(0)); // by default initialize all bytes to zero

        if (!data.empty())
            view_data_ptr = &data[0];
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void VectorBuffer::Resize(size_t num_items) {
        auto new_size = num_items * item_info.item_size;
        auto old_size = data.size();
        data.resize(new_size);
        if (new_size > old_size)
            std::fill(data.begin() + old_size, data.begin() + new_size, char(0));

        // The resize operation can change the pointer to the first element
        if (data.empty())
            view_data_ptr = nullptr;
        else
            view_data_ptr = &data[0];
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void VectorBuffer::Reserve(size_t num_items) {
        data.reserve(num_items + item_info.item_size);
        // The reserve operation can change the pointer to the first element
        if (data.empty())
            view_data_ptr = nullptr;
        else
            view_data_ptr = &data[0];
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void VectorBuffer::ReserveAdditional(size_t num_items) {
        Reserve(num_items + NumItems());
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    VectorBuffer VectorBuffer::DeepCopy(const ItemBuffer &other) {
        VectorBuffer new_buffer(other.GetItemSchema().GetBuilder().Build(), other.item_info.item_size);
        new_buffer.Resize(other.NumItems());
        std::copy(other.view_data_ptr, other.view_data_ptr + other.item_info.item_size * other.NumItems(),
                  new_buffer.data.data());
        return new_buffer;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::unique_ptr<VectorBuffer> VectorBuffer::DeepCopyPtr(const ItemBuffer &other) {
        auto new_buffer = std::make_unique<VectorBuffer>(other.GetItemSchema().GetBuilder().Build(),
                                                         other.item_info.item_size);
        new_buffer->Resize(other.NumItems());
        std::copy(other.view_data_ptr, other.view_data_ptr + other.item_info.item_size * other.NumItems(),
                  new_buffer->data.data());
        return new_buffer;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    VectorBuffer::~VectorBuffer() = default;

    /* -------------------------------------------------------------------------------------------------------------- */
    BufferWrapper::BufferWrapper(ItemSchema &&view, char *buffer, size_t num_items,
                                 int item_size, std::shared_ptr<SmartDataPtrWrapper> smart_ptr) :
            num_items_(num_items),
            smart_data_ptr_(smart_ptr),
            ItemBuffer(std::move(view), item_size) {
        view_data_ptr = buffer;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    BufferWrapper::SmartDataPtrWrapper::~SmartDataPtrWrapper() = default;

} // namespace slam

