#ifndef SLAMCORE_ITERATOR_H
#define SLAMCORE_ITERATOR_H

#include <iterator>

#include "SlamCore/traits.h"
#include "SlamCore/conversion.h"
#include "SlamCore/experimental/iterator/base_iterator.h"

namespace slam {

    /*!
     * @brief   transform iterator maps an iterator to an iterator of a different type
     *          The iterator maps reference of the source type to a reference of the new type via a UnaryFunction
     */
    template<class _UnaryFuncT, class _SourceIteratorT, class _SourceValueT, class _ValueT>
    class transform_iterator :
            public __composition_iterator<transform_iterator<_UnaryFuncT, _SourceIteratorT, _SourceValueT, _ValueT>,
                    _SourceIteratorT, _SourceValueT> {
    public:
        using __parent_t = __composition_iterator<transform_iterator<_UnaryFuncT, _SourceIteratorT, _SourceValueT, _ValueT>,
                _SourceIteratorT, _SourceValueT>;
        typedef convert_pointer_t<_SourceIteratorT, _ValueT> __pointer_t;
        typedef std::iterator_traits<__pointer_t> __std_iterator_traits;
        _UnaryFuncT func;
        typedef typename _SourceIteratorT::iterator_category iterator_category;
        typedef typename _SourceIteratorT::difference_type difference_type;
        typedef typename __std_iterator_traits::value_type value_type;
        typedef typename __std_iterator_traits::pointer pointer;
        typedef typename __std_iterator_traits::reference reference;

        transform_iterator() : __parent_t() {};

        transform_iterator(_SourceIteratorT const &it, _UnaryFuncT f) : func(f), __parent_t(it) {}

        explicit transform_iterator(_SourceIteratorT const &it) : __parent_t(it) {};

        transform_iterator(const transform_iterator &other) : __parent_t(other.source_it), func(other.func) {}

        transform_iterator &operator=(const transform_iterator &other) {
            this->source_it = other.source_it;
            this->func = other.func;
            return *this;
        }

        transform_iterator(transform_iterator &&other) : __parent_t(other.source_it), func(other.func) {
            this->source_it = other.source_it;
        }

        transform_iterator &operator=(transform_iterator &&other) {
            this->source_it = other.source_it;
            this->func = other.func;
            return *this;
        }

        inline reference operator*() const { return func(*this->source_it); }

        inline pointer operator->() const { return &func(*this->source_it); }

        template<typename ReferenceT = reference>
        inline std::enable_if_t<std::is_same_v<ReferenceT, reference> &&
                                std::is_same_v<iterator_category, std::random_access_iterator_tag>, ReferenceT>
        operator[](difference_type __n) const {
            static_assert(std::is_same_v<iterator_category, std::random_access_iterator_tag>);
            return func(this->source_it[__n]);
        }

        inline const pointer base() const { return &func(*(this->source_it)); }

    };


    /*!
     * @brief   Convenient method to make a transform_iterator with template deduction, for a given Conversion
     */
    template<typename _ConversionT, typename _SourceIteratorT>
    slam::transform_iterator<_ConversionT,
            _SourceIteratorT,
            typename _SourceIteratorT::value_type,
            typename _ConversionT::value_type> make_transform(_SourceIteratorT it,
                                                              _ConversionT) {
        static_assert(std::is_same_v<typename _ConversionT::conversion_category, reference_conversion_tag>,
                      "A transform iterator can only be applied with a conversion mapping two references");
        return slam::transform_iterator<_ConversionT, _SourceIteratorT,
                typename _SourceIteratorT::value_type, typename _ConversionT::value_type>(it);
    };

    /**
     * @brief Returns the pair <begin, end> transform iterators of a collection, for a given Conversion
     */
    template<typename _ConversionT, typename _SourceCollection>
    std::pair<slam::transform_iterator<_ConversionT,
            typename _SourceCollection::iterator,
            typename _SourceCollection::iterator::value_type,
            typename _ConversionT::value_type>,
            slam::transform_iterator<_ConversionT,
                    typename _SourceCollection::iterator,
                    typename _SourceCollection::iterator::value_type,
                    typename _ConversionT::value_type>> make_transform_collection(_SourceCollection &collection,
                                                                                  _ConversionT) {
        static_assert(std::is_same_v<typename _ConversionT::conversion_category, reference_conversion_tag>,
                      "A transform iterator can only be applied with a conversion mapping two references");
        return {
                make_transform(collection.begin(), _ConversionT()),
                make_transform(collection.end(), _ConversionT())
        };
    }

    /**
     * @brief Returns the pair <begin, end> transform iterators of a collection, for a given Conversion
     */
    template<typename _ConversionT, typename _SourceCollection>
    std::pair<slam::transform_iterator<_ConversionT,
            typename _SourceCollection::const_iterator,
            typename _SourceCollection::const_iterator::value_type,
            typename _ConversionT::value_type>,
            slam::transform_iterator<_ConversionT,
                    typename _SourceCollection::const_iterator,
                    typename _SourceCollection::const_iterator::value_type,
                    typename _ConversionT::value_type>> make_transform_collection(const _SourceCollection &collection,
                                                                                  _ConversionT) {
        static_assert(std::is_same_v<typename _ConversionT::conversion_category, reference_conversion_tag>,
                      "A transform iterator can only be applied with a conversion mapping two references");
        return {
                make_transform(collection.begin(), _ConversionT()),
                make_transform(collection.end(), _ConversionT())
        };
    }


} // namespace slam

#endif //SLAMCORE_ITERATOR_H
