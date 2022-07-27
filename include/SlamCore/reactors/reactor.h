#ifndef SLAMCORE_REACTOR_H
#define SLAMCORE_REACTOR_H

#include <iostream>
#include <string>
#include <queue>
#include <mutex>
#include <glog/logging.h>
#include <thread>
#include <atomic>

#include "SlamCore/generic_tools.h"
#include "SlamCore/concurrent/blocking_queue.h"

namespace slam {

    /*!
     * @brief   A tag which specifies whether a message type is valid for a given ReActor type
     *
     * This is part of the ReActor programming interface implemented below, and allows
     * Static determination of React method based on the message type.
     *
     * A ReActorT should specialize message_tag::is_valid to true for the messages supported
     * *ie* the messages for which it possess a method `void DoReact(MessageT, message_tag<ReactorT, MessageT>);`
     * With a valid message_tag.
     *
     * @tparam ReActorT     The Derived ReActor class
     * @tparam MessageT     A Message Type which can be valid (is_valid=true) or invalid (is_valid=false) for the ReActorT
     */
    template<typename ReActorT, typename MessageT>
    struct message_tag {
        static const bool is_valid = false;
    };

    // A Syntax sugar to determine if a message_tag is valid
    template<typename ActorT, typename MessageT>
    inline const bool is_message_valid_v = message_tag<ActorT, MessageT>::is_valid;


#define SLAM_REGISTER_MESSAGE(ReactorName, MessageName) \
    template<> struct slam::message_tag<ReactorName, MessageName> { static const bool is_valid = true; };

    /*!
     * @brief   A CRTP base implementation of the (Re)Actor pattern.
     *
     *  The ReActor pattern is a constrained programming paradigm which permits good design compatible in multicore
     *  and asynchronous scenarios limiting the problems of shared memory, in favour of the message passing paradigm.
     *
     *  Derived classes should follow this principle, have private States which are only modified through "DoReact"
     */
    template<typename Derived>
    class GenericReactor {
    public:
        template<typename MessageT>
        std::enable_if_t<is_message_valid_v<Derived, MessageT>, void> React(MessageT message) {
            static_cast<Derived &>(*this).DoReact(message, message_tag<Derived, MessageT>());
        };
    };

    /*!
     * @brief   An abstract Reactor Factory for a specific typed Actor
     *
     * @tparam ReactorT The ReActor type for this factory
     */
    template<typename ReactorT>
    struct AReactorFactory {

        virtual ~AReactorFactory() = 0;

        virtual ReactorT operator()() const = 0;

        virtual bool IsValid() const = 0;
    };

    template<typename ReactorT>
    using ReactorFactoryPtr = std::unique_ptr<AReactorFactory<ReactorT>>;

    template<typename ReactorT>
    AReactorFactory<ReactorT>::~AReactorFactory<ReactorT>() = default;

    /*!
     * @brief   A Trivial Factory for a specific typed Actor having a trivial constructor
     *
     * @tparam ReactorT The ReActor type for this factory
     */
    template<typename ReactorT>
    struct TrivialFactory : AReactorFactory<ReactorT> {

        ReactorT operator()() const override {
            static_assert(std::is_default_constructible_v<ReactorT>,
                          "The reactor is not trivially constructible");
            return ReactorT();
        };

        bool IsValid() const override { return true; }
    };

    template<typename ReactorT>
    std::unique_ptr<TrivialFactory<ReactorT>> MakeTrivialFactory() {
        return std::make_unique<TrivialFactory < ReactorT>>
        ();
    }

    /*!
     * @brief   A Prototype Reactor Factory which creates a Reactor by copying a prototype
     *
     * @tparam ReactorT The ReActor type for this factory
     */
    template<typename ReactorT>
    struct PrototypeFactory : AReactorFactory<ReactorT> {

        ReactorT operator()() const override {
            static_assert(std::is_copy_constructible_v<ReactorT>,
                          "The ReactorT is not copy constructible");
            return ReactorT(prototype);
        }

        bool IsValid() const override { return true; }

        PrototypeFactory() = default;

        explicit PrototypeFactory(const ReactorT &reactor) : prototype(reactor) {}

        explicit PrototypeFactory(ReactorT &&reactor) : prototype(std::move(reactor)) {}

        ReactorT prototype;
    };

    template<typename ReactorT>
    using PrototypeFactoryPtr = std::unique_ptr<PrototypeFactory<ReactorT>>;

    template<typename ReactorT>
    PrototypeFactoryPtr<ReactorT> MakePrototypeFactory() {
        return std::make_unique<PrototypeFactory < ReactorT>>
        ();
    }

    template<typename ReactorT>
    PrototypeFactoryPtr<ReactorT>
    MakePrototypeFactory(const ReactorT &reactor) {
        return std::make_unique<PrototypeFactory < ReactorT>>
        (reactor);
    }

} // namespace slam

#endif //SLAMCORE_REACTOR_H
