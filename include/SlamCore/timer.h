#ifndef SlamCore_TIMER_H
#define SlamCore_TIMER_H

#include <string>
#include <map>
#include <chrono>

namespace slam {

    class Timer {
    public:

        enum UNIT {
            MILLISECONDS,
            SECONDS,
            MINUTES,
            HOURS
        };

        class Ticker {
        public:

            explicit Ticker(Timer &timer, const std::string &entry);

            ~Ticker();


        private:
            typedef std::chrono::high_resolution_clock _clock;
            typedef std::chrono::time_point<_clock> time_t;
            time_t init_time_t;
            std::pair<double, size_t> *pair_ = nullptr;
        };

        void WriteMessage(std::ostream &stream, UNIT unit = SECONDS) const;

        double AverageDuration(const std::string &entry_name, UNIT unit = SECONDS) const;

        double CumulatedDuration(const std::string &entry, UNIT unit = SECONDS) const;

    private:
        std::map<std::string, std::pair<double, size_t>> entries_;

        static inline double MillisecondsToUnit(double ms_duration, UNIT unit) {
            switch (unit) {
                case MILLISECONDS:
                    return ms_duration;
                case SECONDS:
                    return ms_duration / 1000;
                case MINUTES:
                    return ms_duration / (1000 * 60);
                case HOURS:
                    return ms_duration / (1000 * 3600);
                default:
                    throw std::runtime_error("Invalid Time Unit");
            }
        }
    };

}

#endif //SlamCore_TIMER_H
