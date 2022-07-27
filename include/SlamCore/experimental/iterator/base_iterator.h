#ifndef SLAMCORE_BASE_ITERATOR_H
#define SLAMCORE_BASE_ITERATOR_H

#include <iterator>

namespace slam {

    /*!
     * @brief   An incomplete iterator type to iterate over a char buffer, reinterpreted as a`_DestValueT` buffer
     */
    template<class _DerivedIterator>
    class __contiguous_iterator {
    protected:
        char *data_;
        int item_size_;
    public:
        typedef size_t difference_type;
        typedef std::random_access_iterator_tag iterator_category;

        __contiguous_iterator() = default;

        __contiguous_iterator(char *data, int item_size) : data_(data), item_size_(item_size) {}

        inline _DerivedIterator &operator++() {
            data_ += item_size_;
            return static_cast<_DerivedIterator &>(*this);
        }

        inline _DerivedIterator operator++(int) {
            data_ += item_size_;
            return _DerivedIterator(data_, item_size_, static_cast<const _DerivedIterator &>(*this));
        }

        inline _DerivedIterator &operator--() {
            data_ -= item_size_;
            return static_cast<_DerivedIterator &>(*this);
        }

        inline _DerivedIterator operator--(int) {
            data_ -= item_size_;
            return _DerivedIterator(data_, item_size_, static_cast<const _DerivedIterator &>(*this));
        }

        inline _DerivedIterator &operator+=(size_t __n) {
            data_ += __n * item_size_;
            return static_cast<_DerivedIterator &>(*this);
        }

        inline _DerivedIterator operator+(size_t __n) const {

            return _DerivedIterator(data_ + __n * item_size_, item_size_, static_cast<const _DerivedIterator &>(*this));
        }

        inline _DerivedIterator &operator-=(size_t __n) {
            data_ -= __n * item_size_;
            return static_cast<_DerivedIterator &>(*this);
        }

        inline _DerivedIterator operator-(size_t __n) const {
            return _DerivedIterator(data_ - __n * item_size_, item_size_, static_cast<const _DerivedIterator &>(*this));
        }

        inline const char *source_ptr() const {
            return data_;
        };

        inline int item_size() const {
            return item_size_;
        }
    };


    /*!
     * @brief   An incomplete iterator which allows the definition of an iterator built on top of a previous iterator
     */
    template<class _DerivedIterator, class _SourceIteratorT, class _SourceValueT>
    class __composition_iterator {
    public:
        _SourceIteratorT source_it;
        using __iterator_t = _DerivedIterator;
        typedef typename _SourceIteratorT::pointer source_pointer;
        typedef typename _SourceIteratorT::difference_type difference_type;
        typedef typename _SourceIteratorT::iterator_category iterator_category;

#define ITERATOR_STATIC_VERIFICATIONS \
        static_assert(std::is_same_v<typename _SourceIteratorT::value_type, _SourceValueT>, \
                      "Incompatible types between `_SourceValueT` and the `SourceIteratorT::value_type`");

        __composition_iterator() { ITERATOR_STATIC_VERIFICATIONS };

        explicit __composition_iterator(_SourceIteratorT const &it) : source_it(
                it) { ITERATOR_STATIC_VERIFICATIONS };

        inline _DerivedIterator &operator++() {
            source_it++;
            return static_cast<_DerivedIterator &>(*this);
        }

        inline _DerivedIterator operator++(int n) {
            return _DerivedIterator(source_it.operator++(n));
        }

        // Bidirectional iterator requirements
        inline _DerivedIterator &operator--() {
            source_it--;
            return static_cast<_DerivedIterator &>(*this);
        }

        inline _DerivedIterator operator--(int n) { return __iterator_t(source_it.operator--(n)); }

        inline _DerivedIterator &operator+=(difference_type __n) {
            source_it += __n;
            return static_cast<_DerivedIterator &>(*this);
        }

        inline _DerivedIterator operator+(difference_type __n) const { return __iterator_t(source_it + __n); }

        inline _DerivedIterator &operator-=(difference_type __n) {
            source_it -= __n;
            return static_cast<_DerivedIterator &>(*this);
        }

        inline _DerivedIterator operator-(difference_type __n) const { return __iterator_t(source_it - __n); }

        inline const source_pointer source_base() const { return source_it.base(); }

        inline const _SourceIteratorT source_iterator() const { return source_it; }

        static _DerivedIterator make_transform(_SourceIteratorT const &it) {
            return __iterator_t(it);
        }

    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// iterator operations for stl compatibility
#define SLAM_CONTIGUOUS_ITERATOR_BINARY_COMPARISON_OP(op) \
    template<class _DerivedIterator> \
    bool operator op (const __contiguous_iterator<_DerivedIterator>& lhs, \
                      const __contiguous_iterator<_DerivedIterator>& rhs) { \
        return lhs.source_ptr() op rhs.source_ptr();                               \
    }

    SLAM_CONTIGUOUS_ITERATOR_BINARY_COMPARISON_OP(>)

    SLAM_CONTIGUOUS_ITERATOR_BINARY_COMPARISON_OP(<)

    SLAM_CONTIGUOUS_ITERATOR_BINARY_COMPARISON_OP(>=)

    SLAM_CONTIGUOUS_ITERATOR_BINARY_COMPARISON_OP(<=)

    SLAM_CONTIGUOUS_ITERATOR_BINARY_COMPARISON_OP(!=)

    SLAM_CONTIGUOUS_ITERATOR_BINARY_COMPARISON_OP(==)

    template<typename _D>
    typename __contiguous_iterator<_D>::difference_type
    operator-(const __contiguous_iterator<_D> &lhs,
              const __contiguous_iterator<_D> &rhs) {
        CHECK(lhs.item_size() == rhs.item_size());
        return (lhs.source_ptr() - rhs.source_ptr()) / lhs.item_size();
    }

    template<typename _D>
    typename __contiguous_iterator<_D>::difference_type
    operator-(typename __contiguous_iterator<_D>::difference_type __n,
              const __contiguous_iterator<_D> &rhs) {
        return rhs.operator-(__n);
    }

    template<typename _D>
    __contiguous_iterator<_D> operator+(const __contiguous_iterator<_D> &lhs, const __contiguous_iterator<_D> &rhs) {
        CHECK(lhs.item_size() == rhs.item_size());
        return __contiguous_iterator<_D>(
                lhs.source_iterator() + rhs.source_iterator());
    }

    template<typename _D>
    __contiguous_iterator<_D> operator+(typename __contiguous_iterator<_D>::difference_type __n,
                                        const __contiguous_iterator<_D> &rhs) {
        return __contiguous_iterator<_D>(rhs.operator+(__n));
    }



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// iterator operations for stl compatibility
#define SLAM_TRANSFORM_ITERATOR_BINARY_COMPARISON_OP(op) \
    template<class _DerivedIterator, class _SourceIteratorT, class _SourceValueT> \
    bool operator op (const __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>& lhs, \
                      const __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>& rhs) { \
        return lhs.source_iterator() op rhs.source_iterator();                               \
    }

    SLAM_TRANSFORM_ITERATOR_BINARY_COMPARISON_OP(>)

    SLAM_TRANSFORM_ITERATOR_BINARY_COMPARISON_OP(<)

    SLAM_TRANSFORM_ITERATOR_BINARY_COMPARISON_OP(>=)

    SLAM_TRANSFORM_ITERATOR_BINARY_COMPARISON_OP(<=)

    SLAM_TRANSFORM_ITERATOR_BINARY_COMPARISON_OP(!=)

    SLAM_TRANSFORM_ITERATOR_BINARY_COMPARISON_OP(==)

    template<class _DerivedIterator, class _SourceIteratorT, class _SourceValueT>
    typename __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>::difference_type
    operator-(const __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT> &lhs,
              const __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT> &rhs) {
        return lhs.source_iterator() - rhs.source_iterator();
    }

    template<class _DerivedIterator, class _SourceIteratorT, class _SourceValueT>
    typename __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>::difference_type
    operator-(
            typename __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>::difference_type __n,
            const __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT> &rhs) {
        return rhs.source_iterator() - __n;
    }

    template<class _DerivedIterator, class _SourceIteratorT, class _SourceValueT>
    __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>
    operator+(const __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT> &lhs,
              const __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT> &rhs) {
        return __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>(
                lhs.source_iterator() + rhs.source_iterator());
    }

    template<class _DerivedIterator, class _SourceIteratorT, class _SourceValueT>
    __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>
    operator+(
            typename __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>::difference_type __n,
            const __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT> &rhs) {
        return __composition_iterator<_DerivedIterator, _SourceIteratorT, _SourceValueT>(
                rhs.source_iterator() + __n);
    }

} // namespace slam

#endif //SLAMCORE_BASE_ITERATOR_H
