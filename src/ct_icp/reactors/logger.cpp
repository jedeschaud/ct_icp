#include "ct_icp/reactors/logger.h"

namespace ct_icp {

    /* -------------------------------------------------------------------------------------------------------------- */
    std::string Logger::LEVELToString(ct_icp::Logger::LEVEL level) {
        switch (level) {
            case DEBUG:
                return "DEBUG";
            case INFO:
                return "INFO";
            case WARNING:
                return "WARNING";
            case ERROR:
                return "ERROR";
            default:
                return "DEBUG";
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Logger::Log(Logger::LEVEL log_level, const std::string &string_to_log) {
        if (log_level >= level) {
            if (os)
                (*os) << string_to_log;
            else
                SLAM_LOG(INFO) << "[" << LEVELToString(log_level) << "] -- " << string_to_log;
            Notify(string_to_log);
        }
    }

} // namespace ct_icp