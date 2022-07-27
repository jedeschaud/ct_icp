#include "SlamCore/reactors/scheduler.h"


namespace slam {

    /* -------------------------------------------------------------------------------------------------------------- */
    void Scheduler::Run(Scheduler *scheduler) {
        scheduler->is_started = true;
        scheduler->abort = false;

        typedef std::chrono::steady_clock clock_t;
        clock_t::time_point now, after_notify, end;
        auto time = [] { return clock_t::now(); };
        auto begin = time();
        while (!scheduler->abort) {
            now = time();
            {
                auto duration = std::chrono::duration<double, std::ratio<1, 1>>
                        (now - begin).count();
                scheduler->notifier.Notify(duration);
            }
            auto now_bis = time();
            double elapsed = std::chrono::duration<double, std::ratio<1, 1>>
                    (now_bis - now).count();
            double sleep_time_sec = std::max(0., scheduler->sleep_time_sec - elapsed);
            int64_t num_ms = int64_t(sleep_time_sec * 1.e3);
            std::this_thread::sleep_for(std::chrono::milliseconds(num_ms));
        }
        scheduler->is_started = false;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Scheduler::Start() {
        SLAM_CHECK_STREAM(!is_started, "The Scheduler has already started !")
        if (thread) {
            Stop();
        }
        thread = std::make_unique<std::thread>(Scheduler::Run, this);
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    void Scheduler::Stop() {
        abort = true;
        if (thread) {
            thread->join();
            thread = nullptr;
        }
        is_started = false;
    }

} // namespace slam