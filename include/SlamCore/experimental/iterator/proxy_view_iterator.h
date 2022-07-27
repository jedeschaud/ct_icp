#ifndef SLAMCORE_PROXY_VIEW_ITERATOR_H
#define SLAMCORE_PROXY_VIEW_ITERATOR_H

#include <iterator>
#include <glog/logging.h>
#include "SlamCore/data/schema.h"
#include "SlamCore/data/proxy_ref.h"
#include "SlamCore/experimental/iterator/base_iterator.h"

namespace slam {

    /*!
     * @brief       A random_access_iterator, which interprets a char buffer as a buffer of a designed type
     * @tparam T    The type of the object contained into the Item
     */
    template<typename T>
    class proxy_view_iterator : public __contiguous_iterator<proxy_view_iterator<T>> {
    private:
        slam::PROPERTY_TYPE src_property_type_;
        using __parent_t = __contiguous_iterator<proxy_view_iterator<T>>;
        using __parent_t::data_;
        using __parent_t::item_size_;
    public:
        typedef std::random_access_iterator_tag iterator_category;
        typedef size_t difference_type;
        typedef ProxyDRef <T> reference;
        typedef ProxyDRef <T> pointer;
        typedef T value_type;

        proxy_view_iterator() = default;

        proxy_view_iterator(const proxy_view_iterator<T> &other) :
                __parent_t(other.data_, other.item_size_), src_property_type_(other.src_property_type_) {};

        proxy_view_iterator(char *data, int item_size, slam::PROPERTY_TYPE src_pty_type) :
                __parent_t(data, item_size), src_property_type_(src_pty_type) {};

        proxy_view_iterator(char *data, int item_size, const proxy_view_iterator &other) :
                proxy_view_iterator(data, item_size, other.src_property_type_) {};


        // Forward iterator requirements
        reference
        operator*() const { return reference{reinterpret_cast<void *>(data_), src_property_type_}; }

        pointer operator->() const { return operator*(); }

        reference operator[](size_t __n) const { return *reinterpret_cast<pointer>(&data_[__n * item_size_]); }

        const pointer base() const { return reinterpret_cast<pointer>(data_); };
    };


    template<typename T>
    class proxy_view_iterator_const : public __contiguous_iterator<proxy_view_iterator_const<T>> {
    private:
        slam::PROPERTY_TYPE src_property_type_;
        using __parent_t = __contiguous_iterator<proxy_view_iterator_const<T>>;
        using __parent_t::data_;
        using __parent_t::item_size_;
    public:
        typedef std::random_access_iterator_tag iterator_category;
        typedef size_t difference_type;
        typedef const ProxyDRef <T> reference;
        typedef const ProxyDRef <T> pointer;
        typedef T value_type;

        proxy_view_iterator_const() = default;

        proxy_view_iterator_const(const proxy_view_iterator_const<T> &other) :
                __parent_t(other.data_, other.item_size_), src_property_type_(other.src_property_type_) {};

        proxy_view_iterator_const(char *data, int item_size, slam::PROPERTY_TYPE src_pty_type) :
                __parent_t(data, item_size), src_property_type_(src_pty_type) {};

        proxy_view_iterator_const(char *data, int item_size, const proxy_view_iterator_const &other) :
                proxy_view_iterator_const(data, item_size, other.src_property_type_) {};


        // Forward iterator requirements
        reference
        operator*() const { return reference{reinterpret_cast<void *>(data_), src_property_type_}; }

        pointer operator->() const { return operator*(); }

        reference operator[](size_t __n) const { return *reinterpret_cast<pointer>(&data_[__n * item_size_]); }

        const pointer base() const { return reinterpret_cast<pointer>(data_); };
    };


} // namespace slam

#endif //SLAMCORE_PROXY_VIEW_ITERATOR_H
