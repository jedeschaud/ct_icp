#ifndef SLAMCORE_BLOCKING_QUEUE_H
#define SLAMCORE_BLOCKING_QUEUE_H

#include <queue>
#include <mutex>
#include <optional>
#include <condition_variable>
#include "SlamCore/types.h"


namespace slam {

    /*!
     * @brief   A Thread-Safe Blocking Queue for concurrent operations
     *
     * @tparam T  The type of element in the queue
     */
    template<typename T>
    class blocking_queue {
    public:

        blocking_queue() = default;

        blocking_queue(size_t max_capacity) : max_capacity(max_capacity) {}

        // Pushes the item at the back of the queue
        void push(const T &item);

        void emplace(T &&item);

        // Removes all elements from the queue
        void clear();

        // Pops an item from the queue if the queue is not empty, returns {} if it is
        std::optional<T> pop();

        // Waits for the queue to have elements before returning them. If the timeout is exceeded, returns {}
        std::optional<T> blocking_pop(size_t timeout_ms = -1);

        // Similar to blocking_pop but with an additional user-defined release for the condition variable
        template<typename ConditionT>
        std::optional<T> blocking_pop_with(ConditionT release_condition, size_t timeout_ms = -1);

        // Whether the queue is empty
        bool empty() const;

        // Returns the queue size
        size_t size() const;

        // Sends a notification for user defined conditions to be verified
        // Note that this method is only relevant if `blocking_pop_with` was called
        void notify_event();

        // Sets the max capacity of the blocking_queue
        void set_max_capacity(size_t capacity) { max_capacity = capacity; }

    private:
        std::atomic<size_t> max_capacity = slam::kInvalidIndex;
        std::queue<T> queue_;
        std::condition_variable cv_;
        mutable std::mutex mutex_;
    };


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS                                                                                              ///
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void blocking_queue<T>::push(const T &item) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            queue_.push(item);
            while (queue_.size() > max_capacity)
                queue_.pop();
        }
        cv_.notify_one();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void blocking_queue<T>::emplace(T &&item) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            queue_.emplace(std::move(item));
            while (queue_.size() > max_capacity)
                queue_.pop();
        }
        cv_.notify_one();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    std::optional<T> blocking_queue<T>::pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.empty())
            return {};
        T front = std::move(queue_.front());
        queue_.pop();
        return front;
    }

    /* -------------------------------------------------------------------------------------------------------------- */

#define BLOCKING_OP_MEMBER \
        if (timeout_ms == slam::kInvalidIndex) { \
            cv_.wait(lock, check_cv); \
        } else { \
            auto status = cv_.wait_for(lock, timeout_ms * 1ms, check_cv); \
        }                                   \
        if(queue_.empty())                  \
            return {};                      \
        T front = std::move(queue_.front()); \
        queue_.pop(); \
        return front;


    template<typename T>
    std::optional<T> blocking_queue<T>::blocking_pop(size_t timeout_ms) {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lock(mutex_);
        auto check_cv = [&] { return !queue_.empty(); };
        BLOCKING_OP_MEMBER
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    template<typename ConditionT>
    std::optional<T> blocking_queue<T>::blocking_pop_with(ConditionT release_condition, size_t timeout_ms) {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lock(mutex_);
        auto check_cv = [&] {
            return release_condition() || !queue_.empty();
        };
        BLOCKING_OP_MEMBER
    }


    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    bool blocking_queue<T>::empty() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    size_t blocking_queue<T>::size() const {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.size();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void blocking_queue<T>::notify_event() {
        cv_.notify_one();
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename T>
    void blocking_queue<T>::clear() {
        std::unique_lock<std::mutex> lock{mutex_};
        while (!queue_.empty())
            queue_.pop();
    }
    /* -------------------------------------------------------------------------------------------------------------- */

}

#endif //SLAMCORE_BLOCKING_QUEUE_H
