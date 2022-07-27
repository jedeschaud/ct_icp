#ifndef SLAMCORE_VIEW_ITERATOR_H
#define SLAMCORE_VIEW_ITERATOR_H

#include <iterator>
#include <glog/logging.h>

#include "SlamCore/experimental/iterator/base_iterator.h"

namespace slam {

    /*!
     * @brief       A random_access_iterator, which interprets a char buffer as a buffer of a designed type
     * @tparam T    The type of the object contained into the Item
     */
    template<typename T>
    class view_iterator : public __contiguous_iterator<view_iterator<T>> {
    protected:
        using __parent_t = __contiguous_iterator<view_iterator<T>>;
        using __parent_t::data_;
        using __parent_t::item_size_;
        typedef T *__pointer_t;
        typedef std::iterator_traits<__pointer_t> __std_iterator_traits;
    public:
        typedef std::random_access_iterator_tag iterator_category;
        typedef typename __std_iterator_traits::difference_type difference_type;
        typedef typename __std_iterator_traits::reference reference;
        typedef typename __std_iterator_traits::pointer pointer;
        typedef typename __std_iterator_traits::value_type value_type;

        view_iterator() = default;

        view_iterator(const view_iterator<T> &other) : __parent_t(other.data_, other.item_size_) {};

        explicit view_iterator(T *&_i, int item_size) : __parent_t((char *) (_i),
                                                                   item_size) {
            CHECK(sizeof(T) <= item_size_);
        }

        view_iterator(char *data_ptr, int item_size) : __parent_t(data_ptr, item_size) {};

        view_iterator(T *&_i, int item_size, const view_iterator &) : view_iterator(_i, item_size) {}

        view_iterator(char *data, int item_size, const view_iterator &) : __parent_t(data, item_size) {}

        // Forward iterator requirements
        reference
        operator*() const { return *reinterpret_cast<T *>(data_); }

        pointer
        operator->() const { return reinterpret_cast<T *>(data_); }

        reference operator[](size_t __n) const { return *reinterpret_cast<pointer>(&data_[__n * item_size_]); }

        const pointer base() const { return reinterpret_cast<pointer>(data_); };
    };


} // namespace slam

#endif //SLAMCORE_VIEW_ITERATOR_H
