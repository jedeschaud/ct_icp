#include <SlamCore/timer.h>
#include <glog/logging.h>

namespace slam {


    /* -------------------------------------------------------------------------------------------------------------- */
    void Timer::WriteMessage(std::ostream &stream, Timer::UNIT unit) const {

        stream << "********************************************" << std::endl
               << "[TIMER] All Active Clocks" << std::endl;
        std::string unit_str;
        switch (unit) {
            case MILLISECONDS:
                unit_str = "(ms)";
                break;
            case SECONDS:
                unit_str = "(s)";
                break;
            case MINUTES:
                unit_str = "(min)";
                break;
            case HOURS:
                unit_str = "(hrs)";
                break;
        }
        for (auto &pair: entries_) {
            auto &entry = pair.second;
            auto num_ticks = std::get<1>(entry);
            double accum_duration = MillisecondsToUnit(entry.first, unit);
            double avg_duration = num_ticks > 0 ? accum_duration / num_ticks : 0;

            stream << pair.first << ": Average Duration " << avg_duration << unit_str << ", Total Duration "
                   << accum_duration << unit_str << ", Number of Ticks: " << num_ticks << std::endl;
        }
        stream << "********************************************" << std::endl;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    double Timer::AverageDuration(const std::string &entry_name, Timer::UNIT unit) const {
        CHECK(entries_.find(entry_name) != entries_.end()) << "Entry " << entry_name << " does not exist in timer";
        const auto &entry = entries_.at(entry_name);
        return MillisecondsToUnit(entry.first, unit) / entry.second;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    double Timer::CumulatedDuration(const std::string &entry_name, Timer::UNIT unit) const {
        CHECK(entries_.find(entry_name) != entries_.end()) << "Entry " << entry_name << " does not exist in timer";
        const auto &entry = entries_.at(entry_name);
        return MillisecondsToUnit(std::get<1>(entry), unit);
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    Timer::Ticker::~Ticker() {
        if (pair_) {
            auto current_t = _clock::now();
            auto duration_ms = std::chrono::duration<double, std::milli>(current_t - init_time_t).count();
            pair_->first += duration_ms;
            pair_->second += 1;
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    Timer::Ticker::Ticker(Timer &timer, const std::string &entry) {
        init_time_t = _clock::now();
        if (timer.entries_.find(entry) == timer.entries_.end()) {
            timer.entries_[entry] = {0, 0};
        }
        pair_ = &timer.entries_[entry];
    }
} // namespace slam

