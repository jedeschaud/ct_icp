#ifndef SLAMCORE_TRAITS_H
#define SLAMCORE_TRAITS_H

namespace slam {


    /* ----------------------------------------------------------------------------------------------- */
    /* -- Trait to build a pointer with/without a const property mimicking a based iterator/pointer -- */
    template<typename _SourceIterator, typename _DestT>
    struct convert_pointer {
        typedef typename convert_pointer<typename _SourceIterator::pointer, _DestT>::type type;
    };

    template<typename _SourceT, typename _DestT>
    struct convert_pointer<_SourceT *, _DestT> {
        typedef _DestT *type;
    };

    template<typename _SourceT, typename _DestT>
    struct convert_pointer<const _SourceT *, _DestT> {
        typedef const _DestT *type;
    };

    template<typename _SourceIterator, typename _DestT>
    using convert_pointer_t = typename convert_pointer<_SourceIterator, _DestT>::type;


} // namespace slam

#endif //SLAMCORE_TRAITS_H
