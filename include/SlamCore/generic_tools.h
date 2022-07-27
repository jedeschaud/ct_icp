#ifndef SlamCore_GENERIC_TOOLS_H
#define SlamCore_GENERIC_TOOLS_H

#include <vector>
#include <algorithm>

namespace slam {

#define CONST_REF_GETTER(name, path) \
    inline const auto& name ## Const() const { return path;}

#define REF_GETTER(name, path) \
    inline auto& name() {return path;}; \
    CONST_REF_GETTER(name, path)

#define GETTER(name, path) \
    inline const auto& Get ## name() const {return path;}

#define SETTER(name, path, type) \
    void Set ## name (type item) { path = item; }

#define GETTER_SETTER(name, path, type) \
    GETTER(name, path)            \
    SETTER(name, path, type)

    /*!
     * A Templated utility function which applies a transformation from a vector into another of a different type
     */
    template<typename T, typename AllocT_=std::allocator<T>>
    auto transform_vector = [](const auto &vector, auto &&conversion) {
        std::vector<T, AllocT_> result(vector.size());
        std::transform(vector.begin(), vector.end(), result.begin(), conversion);
        return result;
    };


    /*!
     * A logger with a static variable which logs only one message every `frequency` calls
     * It is useful not to pollute the log files
     */
#define STATIC_LOGGER(logger, frequency, message) \
{\
    static int _call_idx = 0; \
    if (_call_idx++ % frequency == 0)     \
        logger << "[LOGGER CALLED " << _call_idx << " times]. Message:" << message;\
}


} // namespace slam

#endif //SlamCore_GENERIC_TOOLS_H
