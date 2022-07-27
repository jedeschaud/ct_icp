#ifndef SLAMCORE_OBSERVER_H
#define SLAMCORE_OBSERVER_H

#include <memory>

namespace slam {

    /*!
     * @brief   An Observer is an abstract and templated implementation of the Observer Design Pattern
     *
     * An observer simply reacts to a notification, in the programming pattern, a Subject/Notifier
     * is assigned a collection of observers, which are notified during the program flow, calling the OnNotify method.
     *
     * @tparam T    The type of notification the observer responds to
     */
    template<typename T>
    struct Observer {

        virtual void OnNotify(T notification) = 0;

        virtual ~Observer() = default;

    };

    template<typename T>
    using ObserverPtr = std::shared_ptr<Observer<T>>;

    /*!
     * @brief   A LambdaObserver is an observer wrapping a Lambda function
     *
     * @tparam T        The input type of the lambda function
     * @tparam LambdaT  The type of the lambda function
     */
    template<typename T, typename LambdaT>
    struct LambdaObserver : Observer<T> {

        explicit LambdaObserver(LambdaT &&_lambda) : lambda(std::forward<LambdaT>(_lambda)) {}

        void OnNotify(T notification) override {
            return lambda(notification);
        }

        ~LambdaObserver() = default;

        LambdaT lambda;
    };

    // Syntax Sugar to create a LambdaObserver by specifying only the type of the input only
    // e.g. MakeObserver<int>([](int x) { std::cout << x << std::endl; });
    template<typename T, typename LambdaT>
    ObserverPtr<T> MakeObserver(LambdaT &&lambda) {
        return std::make_shared<LambdaObserver<T, LambdaT>>(std::forward<LambdaT>(lambda));
    }

}

#endif //SLAMCORE_OBSERVER_H
