#ifndef CT_ICP_LOGGER_H
#define CT_ICP_LOGGER_H
#include <string>

#include <SlamCore/reactors/notifier.h>
#include <SlamCore/utils.h>


namespace ct_icp {

#ifndef CT_ICP_DEFAULT_LOG
#define CT_ICP_DEFAULT_LOG 0
#endif

    /**
     * @brief A Logger is a specialized Notifier which broadcasts formatted log messages to observers
     */
    struct Logger : slam::Notifier<std::string> {
        enum LEVEL {
            DEBUG = 0,
            INFO = 1,
            WARNING = 2,
            ERROR = 3
        };

        static std::string LEVELToString(LEVEL level);

        void Log(LEVEL log_level, const std::string &string_to_log);

        explicit Logger(std::ostream *os = nullptr) : os(os) {}

        LEVEL level = LEVEL(CT_ICP_DEFAULT_LOG);
        std::ostream *os = nullptr;
    };
}


#endif //CT_ICP_LOGGER_H
