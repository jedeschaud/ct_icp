#ifndef SLAMCORE_NOTIFIER_H
#define SLAMCORE_NOTIFIER_H
#include <list>
#include <map>
#include <algorithm>

#include <SlamCore/reactors/observer.h>

namespace slam {


    // The index type of an observer in a Notifier
    typedef size_t observer_id_t;

    /*!
     * @brief   A Notifier notifies a collection of Observers of an event in the implementation of the Observer Pattern
     *
     * @tparam T    The type of the notification
     */
    template<typename T>
    struct Notifier {

        // Adds an Observer wrapping a Lambda, returns the idx of the observer
        template<typename LambdaT>
        observer_id_t AddObserverLambda(LambdaT &&lambda) {
            return AddObserver(MakeObserver<T>(std::forward<LambdaT>(lambda)));
        }

        // Adds an observer, returns the idx of the observer
        observer_id_t AddObserver(ObserverPtr<T> &&observer) {
            observers.emplace(observer_idx_, std::move(observer));
            return observer_idx_++;
        }

        // Removes an observer from the container
        void RemoveObserver(observer_id_t observer_id) {
            observers.erase(observer_id);
        }

        // Removes all observers
        void Clear() {
            observers.clear();
        }

        // Notify all observers
        void Notify(const T &item) {
            std::for_each(observers.begin(),
                          observers.end(),
                          [&item](auto &pair) {
                              if (pair.second)
                                  pair.second->OnNotify(item);
                          });
        }

        // Returns whether the Notifier has observers
        bool HasObservers() {
            return !observers.empty();
        }

        Notifier() = default;

    private:
        std::map<size_t, ObserverPtr<T>> observers;
        size_t observer_idx_ = 0;
    };

}

#endif //SLAMCORE_NOTIFIER_H
