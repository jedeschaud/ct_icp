#ifndef SLAMCORE_SCHEDULER_H
#define SLAMCORE_SCHEDULER_H

#include <thread>
#include <chrono>
#include <atomic>

#include "SlamCore/utils.h"
#include "SlamCore/reactors/notifier.h"

namespace slam {

    /**
     * @brief A Scheduler to periodically notifies observers
     */
    struct Scheduler {

        /** Starts the scheduler in a new thread which notifies observers periodically */
        void Start();

        /** Stops the Scheduler (waits for the thread to complete) */
        void Stop();

        /** Signal the scheduler to abort its main loop */
        inline void Abort() {
            abort = true;
        }

        /** Signal the scheduler to abort its main loop */
        inline bool IsRunning() const {
            return thread && is_started;
        }

        /**
         * Adds a lambda to be notified
         * @tparam LambdaT type of the lambda (typically deduced automatically)
         * @returns The id of the observer
         */
        template<typename LambdaT>
        observer_id_t AddObserverLambda(LambdaT &&lambda) {
            return AddObserver(MakeObserver<double>(std::forward<LambdaT>(lambda)));
        }

        /** @brief Adds an observer @returns the idx of the observer */
        observer_id_t AddObserver(ObserverPtr<double> &&observer) {
            SLAM_CHECK_STREAM(!is_started, "Cannot add an observer after the Scheduler has started.")
            return notifier.AddObserver(std::move(observer));
        }


        /** Sets the frequency of notification (in Hz) */
        inline void SetFrequency(double frequency) {
            sleep_time_sec = 1. / frequency;
        }

        explicit Scheduler(double frequency_htz = 100) : sleep_time_sec(1. / frequency_htz) {}

        ~Scheduler() {
            Stop();
        }

    private:
        static void Run(Scheduler *scheduler);

        std::unique_ptr<std::thread> thread = nullptr;
        slam::Notifier<double> notifier;
        std::atomic<double> sleep_time_sec = 0.1;
        std::atomic<bool> is_started = false;
        std::atomic<bool> abort = false;
    };

} // namespace slam

#endif //SLAMCORE_SCHEDULER_H
