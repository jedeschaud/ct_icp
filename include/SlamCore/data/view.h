#ifndef SlamCore_VIEW_H
#define SlamCore_VIEW_H

#include <iterator>
#include "SlamCore/data/buffer.h"
#include "SlamCore/data/proxy_ref.h"
#include "SlamCore/experimental/iterator/view_iterator.h"
#include "SlamCore/experimental/iterator/proxy_view_iterator.h"


namespace slam {


    /*!
     * @brief   A View provides a typed fixed size vector-like interface to the data of an item buffer
     *
     * @tparam T    The type interpretation of the bytes in the buffer
     */
    template<typename T>
    struct View {

        View(ItemBuffer &_buffer_view, int _offset_in_item, int _item_size) :
                item_size(_item_size), item_buffer(_buffer_view), offset_in_item(_offset_in_item) {};

        // Returns a reference of an item element at `index`
        T &operator[](size_t index);

        // Returns a const reference of an item element at `index`
        const T &operator[](size_t index) const;

        // Returns a reference to the item in the front
        T &front();

        // Returns a reference to the item in the front
        const T &front() const;

        // Returns a reference to the item at the back
        T &back();

        // Returns a reference to the item at the back
        const T &back() const;

        // Returns an iterator to the beginning
        view_iterator<T> begin();

        // Returns a const iterator to the beginning
        view_iterator<const T> cbegin() const;

        // Returns an iterator to the end
        view_iterator<T> end();

        // Returns a const iterator to the end
        view_iterator<const T> cend() const;

        // Whether the view is empty
        bool empty() const;

        // Number of items in the view
        size_t size() const;

        const int offset_in_item;       // The offset in the item layout
        const int item_size;            // The size of the item
        ItemBuffer &item_buffer;        // The buffer pointing to the real data
    };

    /*!
     * @brief   A ResizableView provides a typed resizable vector-like interface to the data of an item buffer
     *
     * @tparam T    The type interpretation of the bytes in the buffer
     *
     * @note    For each insertion made in the buffer, the buffer will add as many bytes as the size of underlying Item
     *          And not the Representation type.
     */
    template<typename T>
    struct ResizableView : View<T> {

        ResizableView(ResizableBuffer &_buffer_view, int _offset_in_item, int _item_size) :
                View<T>(dynamic_cast<ItemBuffer &>(_buffer_view), _offset_in_item, _item_size),
                resizable_buffer(_buffer_view) {};


        // Resizes the Item buffer
        void resize(size_t size);

        // Reserves space for `new_size` items
        void reserve(size_t new_size);

        // Pushes back an item at the end of the buffer
        void push_back(const T &new_item);

        ResizableBuffer &resizable_buffer;
    };


    /*!
     * @brief   Returns a proxy view
     */
    template<typename DestT>
    struct ProxyView {
        using ProxyType = ProxyDRef<DestT>;

        ProxyView(slam::PROPERTY_TYPE source_property_type,
                  slam::ItemBuffer &_buffer_view, int _offset_in_item, int _item_size) :
                src_property_type(source_property_type),
                item_size(_item_size), item_buffer(_buffer_view), offset_in_item(_offset_in_item) {};

        // Number of items in the view
        size_t size() const {
            return item_buffer.NumItems();
        }

        // Returns a Proxy for the element/property of the item at index `index`
        ProxyType operator[](size_t index) {
            char *elem_ptr = item_buffer.view_data_ptr + (index * item_size) + offset_in_item;
            return ProxyType{reinterpret_cast<void *>(elem_ptr), src_property_type};
        }

        // Returns a const Proxy for the element/property of the item at index `index`
        const ProxyType operator[](size_t index) const {
            auto *elem_ptr = const_cast<char *>(item_buffer.view_data_ptr + (index * item_size) + offset_in_item);
            return ProxyType{reinterpret_cast<void *>(elem_ptr), src_property_type};
        }

        // Returns a reference to the item in the front
        ProxyType front() {
            return operator[](0);
        };

        // Returns a reference to the item in the front
        const ProxyType front() const {
            return operator[](0);
        };

        // Returns a reference to the item at the back
        ProxyType back() {
            return operator[](size() - 1);
        };

        // Returns a reference to the item at the back
        const ProxyType back() const {
            return operator[](size() - 1);
        };

        // Returns an iterator to the beginning
        proxy_view_iterator<DestT> begin() {
            return proxy_view_iterator<DestT>(item_buffer.view_data_ptr + offset_in_item,
                                              item_size, src_property_type);
        };

        // Returns an iterator to the beginning
        const proxy_view_iterator<DestT> begin() const {
            return proxy_view_iterator<DestT>(item_buffer.view_data_ptr + offset_in_item,
                                              item_size, src_property_type);
        };

        // Returns a const iterator to the beginning
        proxy_view_iterator_const<DestT> cbegin() const {
            return proxy_view_iterator_const<DestT>(item_buffer.view_data_ptr + offset_in_item,
                                                    item_size, src_property_type);
        };

        // Returns an iterator to the end
        proxy_view_iterator<DestT> end() {
            return proxy_view_iterator<DestT>(item_buffer.view_data_ptr + offset_in_item + item_size * size(),
                                              item_size, src_property_type);
        };

        // Returns an iterator to the end
        const proxy_view_iterator<DestT> end() const {
            return proxy_view_iterator<DestT>(item_buffer.view_data_ptr + offset_in_item + item_size * size(),
                                              item_size, src_property_type);
        };

        // Returns a const iterator to the end
        proxy_view_iterator_const<DestT> cend() const {
            return proxy_view_iterator_const<DestT>(item_buffer.view_data_ptr + offset_in_item + item_size * size(),
                                                    item_size, src_property_type);
        };

        // Whether the view is empty
        bool empty() const {
            return size() <= 0;
        };


        slam::PROPERTY_TYPE src_property_type;
        const int offset_in_item;       // The offset in the item
        const int item_size;            // The item size
        slam::ItemBuffer &item_buffer;
    };



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    T &View<T>::operator[](size_t index) {
        char *elem_ptr = item_buffer.view_data_ptr + (index * item_size) + offset_in_item;
        return *reinterpret_cast<T *>(elem_ptr);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const T &View<T>::operator[](size_t index) const {
        const char *elem_ptr = item_buffer.view_data_ptr + (index * item_size) + offset_in_item;
        return *reinterpret_cast<const T *>(elem_ptr);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    T &View<T>::front() {
        char *elem_ptr = item_buffer.view_data_ptr + offset_in_item;
        return *reinterpret_cast<T *>(elem_ptr);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    T &View<T>::back() {
        char *elem_ptr = item_buffer.view_data_ptr + ((item_buffer.NumItems() - 1) * item_size) + offset_in_item;
        return *reinterpret_cast<T *>(elem_ptr);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    size_t View<T>::size() const {
        return item_buffer.NumItems();
    }

#define VIEW_ITERATOR_CHECK_SIZE CHECK(sizeof(T) + offset_in_item <= item_size);

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    view_iterator<T> View<T>::begin() {
        VIEW_ITERATOR_CHECK_SIZE
        T *ptr = reinterpret_cast<T *>(item_buffer.view_data_ptr + offset_in_item);
        return view_iterator<T>(ptr, item_size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    view_iterator<const T> View<T>::cbegin() const {
        VIEW_ITERATOR_CHECK_SIZE
        const T *ptr = reinterpret_cast<const T *>(item_buffer.view_data_ptr + offset_in_item);
        return view_iterator<const T>(ptr, item_size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    view_iterator<T> View<T>::end() {
        VIEW_ITERATOR_CHECK_SIZE
        T *ptr = reinterpret_cast<T *>(item_buffer.view_data_ptr + offset_in_item + item_size * size());
        return view_iterator<T>(ptr, item_size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    view_iterator<const T> View<T>::cend() const {
        VIEW_ITERATOR_CHECK_SIZE
        const T *ptr = reinterpret_cast<const T *>(item_buffer.view_data_ptr + offset_in_item + item_size * size());
        return view_iterator<const T>(ptr, item_size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    bool View<T>::empty() const {
        return cbegin() == cend();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const T &View<T>::front() const {
        return *cbegin();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    const T &View<T>::back() const {
        CHECK(size() > 1);
        return *cend() - 1;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void ResizableView<T>::resize(size_t size) {
        resizable_buffer.Resize(size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void ResizableView<T>::reserve(size_t new_size) {
        resizable_buffer.Reserve(new_size);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void ResizableView<T>::push_back(const T &new_item) {
        resizable_buffer.InsertItems(1, nullptr);
        char *last_data = resizable_buffer.BackPtr();
        const char *new_item_data = reinterpret_cast<const char *>(&new_item);
        for (auto i(0); i < sizeof(T); ++i)
            last_data[i + static_cast<View<T>>(this)->offset_in_item] = new_item_data[i];
    }


} // namespace slam

#endif //SlamCore_VIEW_H
