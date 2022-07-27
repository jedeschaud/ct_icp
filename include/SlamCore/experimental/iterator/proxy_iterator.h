#ifndef SLAMCORE_PROXY_ITERATOR_H
#define SLAMCORE_PROXY_ITERATOR_H

#include "SlamCore/data/proxy_ref.h"
#include "SlamCore/experimental/iterator/base_iterator.h"

namespace slam {

    /*!
     * @brief   proxy_iterator allows to build an iterator with proxy reference to a given type
     *
     * @note    proxy_iterator provides very little support for the stl, and any use beyond simple
     *          iteration of its element is discouraged
     */
    template<class _SourceIteratorT, class _SourceValueT, class _DestValueT, class _ConversionT>
    class proxy_iterator :
            public __composition_iterator<proxy_iterator<_SourceIteratorT, _SourceValueT, _DestValueT, _ConversionT>,
                    _SourceIteratorT, _SourceValueT> {
    private:
        using __parent_t = __composition_iterator<proxy_iterator<_SourceIteratorT, _SourceValueT, _DestValueT, _ConversionT>,
                _SourceIteratorT, _SourceValueT>;
        using __parent_t::source_it;
        _ConversionT conversion_;
    public:
        typedef typename _SourceIteratorT::iterator_category iterator_category;
        typedef typename _SourceIteratorT::difference_type difference_type;
        typedef _DestValueT value_type;
        typedef ProxySRef<_DestValueT, _SourceValueT, _ConversionT> reference;
        typedef ProxySRef<_DestValueT, _SourceValueT, _ConversionT> pointer;

        explicit proxy_iterator(_SourceIteratorT const &it) : __parent_t(it) {};

        explicit proxy_iterator(_SourceIteratorT const &it, _ConversionT _conversion)
                : __parent_t(it), conversion_(_conversion) {};

        reference operator*() const { return reference(*source_it); }

        pointer operator->() const { return pointer(source_it); }

        // Random access iterator requirements
        std::enable_if_t<std::is_same_v<typename _SourceIteratorT::iterator_category,
                std::random_access_iterator_tag>, reference> operator[](size_t __n) const {
            return reference(source_it[__n]);
        }

        const pointer base() const { return pointer(*source_it.base()); }

    };


} // namespace slam

#endif //SLAMCORE_PROXY_ITERATOR_H
