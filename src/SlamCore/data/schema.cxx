#include <numeric>

#include "SlamCore/data/schema.h"

namespace slam {


    /* -------------------------------------------------------------------------------------------------------------- */
    ItemInfo::ItemInfo(ItemSchema &&schema,
                       ItemBuffer &buffer, int item_size) : item_schema(std::move(schema)),
                                                            parent_buffer(&buffer),
                                                            item_size(item_size) {
        for (auto &element: item_schema.element_infos_)
            const_cast<ElementInfo &>(element.second).parent = this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool ItemInfo::HasElement(const std::string &element_name) const {
        return item_schema.HasElement(element_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    int ElementInfo::ElementSize() const {
        return std::accumulate(properties.begin(),
                               properties.end(),
                               0, [](int acc, const PropertyInfo &info) {
                    return acc + info.Size();
                });
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void ElementInfo::CheckMemoryLayout() const {
        std::vector<bool> data_bytes(ElementSize(), false);
        for (auto &property: properties) {
            int base_byte = property.offset_in_elem;
            int num_bytes = PropertySize(property.type) * property.dimension;
            for (auto i(base_byte); i < base_byte + num_bytes; ++i) {
                CHECK(i < data_bytes.size() && i >= 0)
                                << "Property error: the layout of the data is bigger than the element size."
                                << std::endl
                                << "This can be caused by non contiguous properties inside an element."
                                << std::endl;
                CHECK(!data_bytes[i]) << "Properties error : the layout of two properties overlap" << std::endl;
                data_bytes[i] = true;
            }
        }
        CHECK(std::all_of(data_bytes.begin(),
                          data_bytes.end(), [](auto x) { return x; }))
                        << "The memory layout of the element is not contiguous, there are holes";
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ElementInfo::ElementInfo(std::vector<PropertyInfo> &&_properties, int _offset_in_item, std::string &&_element_name)
            : properties(std::move(_properties)),
              offset_in_item(_offset_in_item),
              element_name(std::move(_element_name)) {
        for (auto &property: properties)
            property.parent = this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool ElementInfo::HasProperty(const std::string &property_name) const {
        return std::find_if(properties.begin(), properties.end(),
                            [&property_name](const PropertyInfo &property) {
                                return property.property_name == property_name;
                            }) != properties.end();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    const PropertyInfo &ElementInfo::GetProperty(const std::string &property_name) const {
        CHECK(HasProperty(property_name)) << "The property " << property_name
                                          << " does not exist in the element" << std::endl;
        return *std::find_if(properties.begin(), properties.end(),
                             [&property_name](const PropertyInfo &property) {
                                 return property.property_name == property_name;
                             });
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema::Builder &ItemSchema::Builder::AddElement(const std::string &element_name, int offset_in_item) {
        element_infos_[element_name] = {slam::ElementInfo{{}, offset_in_item, std::string(element_name)}};
        return *this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    slam::ElementInfo &ItemSchema::Builder::GetElement(const std::string &element_name) {
        CHECK(element_infos_.find(element_name) != element_infos_.end())
                        << "The Builder has no element with name " << element_name << std::endl;
        return element_infos_[element_name];
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema::Builder &ItemSchema::Builder::AddProperty(const std::string &element_name, std::string &&property_name,
                                                          slam::PROPERTY_TYPE ptype, int offset_in_elem, int dim) {
        CHECK(element_infos_.find(element_name) !=
              element_infos_.end()) << "The Builder does not contain the element " << element_name << std::endl;
        element_infos_[element_name].properties.push_back(
                {dim, offset_in_elem, ptype, std::move(property_name), nullptr});
        return *this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema ItemSchema::Builder::Build() const {
        CHECK(item_size_ > 0) << "The item size is not set to a positive value" << std::endl;
        // Check the layout of the data
        for (auto &element: element_infos_) {
            element.second.CheckMemoryLayout();
            CHECK(element.second.offset_in_item + element.second.ElementSize() <= item_size_)
                            << "The element layout is not contained in the item. Incompatible layout" << std::endl;
        }


        return ItemSchema(element_infos_, item_size_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema::Builder::Builder(const std::map<std::string, ElementInfo> &element_infos, int item_size) :
            element_infos_(element_infos), item_size_(item_size) {}

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema::Builder::Builder(size_t item_size) : item_size_(item_size) {}

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema::Builder &ItemSchema::Builder::SetItemSize(int item_size) {
        item_size_ = item_size;
        return *this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema::Builder &ItemSchema::Builder::RemoveElement(const std::string &element_name) {
        if (element_infos_.find(element_name) != element_infos_.end())
            element_infos_.erase(element_name);
        return *this;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema::Builder ItemSchema::GetBuilder() const {
        return Builder(element_infos_, item_size_);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool ItemSchema::HasProperty(const std::string &element_name, const std::string &property_name) const {
        if (!HasElement(element_name)) {
            LOG(INFO) << "The element " << element_name << " does not exist in the schema." << std::endl;
            return false;
        }
        const auto &elem_info = element_infos_.at(element_name);
        return elem_info.HasProperty(property_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    const PropertyInfo &
    ItemSchema::GetPropertyInfo(const std::string &element_name, const std::string &property_name) const {
        const auto &elem_info = element_infos_.at(element_name);
        return elem_info.GetProperty(property_name);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    bool ItemSchema::HasElement(const std::string &name) const {
        return element_infos_.find(name) != element_infos_.end();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::ostream &operator<<(std::ostream &os, const ItemSchema &schema) {
        os << "Item Size: " << schema.item_size_ << std::endl;
        os << "Elements Layout:" << std::endl;
        for (auto &elem: schema.element_infos_) {
            os << '\t' << elem.first << ":" << " [offset in item:" << elem.second.offset_in_item << "]" << std::endl;
            for (const auto &pty: elem.second.properties) {
                os << ("\t\t") << pty.property_name << ": [type:" << PropertyTypeChar(pty.type)
                   << " / offset in elem:" << pty.offset_in_elem << " / dim:"
                   << pty.dimension << "]" << std::endl;
            }
        }

        return os;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    ItemSchema::ItemSchema(const std::map<std::string, ElementInfo> &element_infos, int item_size) :
            element_infos_(element_infos), item_size_(item_size) {
        for (auto &pair: element_infos_) {
            for (auto &pty: pair.second.properties)
                pty.parent = &pair.second;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    int ItemSchema::GetTotalElementSize() const {
        return std::accumulate(element_infos_.begin(),
                               element_infos_.end(), 0,
                               [](int acc, const std::pair<std::string, ElementInfo> &pair) {
                                   return acc + pair.second.ElementSize();
                               });
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    std::vector<std::string> ItemSchema::GetElementNames() const {
        std::vector<std::string> element_names;
        for (auto &elem_info: element_infos_)
            element_names.push_back(elem_info.first);
        return element_names;
    }

}

