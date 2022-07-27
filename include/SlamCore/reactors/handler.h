#ifndef SLAMCORE_HANDLER_H
#define SLAMCORE_HANDLER_H

#include <SlamCore/reactors/reactor.h>
#include <SlamCore/utils.h>

namespace slam {
    /*!
     * @brief A Handler constructs a ReActor in an event loop, and handles messages,
     *        sending them to the ReActor.
     *
     * @tparam ReActorT The type of ReActor constructed using a Factory
     * @tparam MessageT
     */
    template<typename ReActorT, typename MessageT,
            typename _ = std::enable_if_t<is_message_valid_v<ReActorT, MessageT>>>
    class Handler {
    public:
        typedef Handler<ReActorT, MessageT, _> this_type;

        static void Run(this_type *self) {
            {
                std::lock_guard<std::mutex> lock{self->factory_mutex};
                SLAM_CHECK_STREAM(!self->is_running, "The Handler is already running !");
                self->is_running = true;
            }

            SLAM_CHECK_STREAM(self->reactor_factory_ && self->reactor_factory_->IsValid(),
                          "[EventLoopReActorWrapper] The Reactor Factory is an Invalid State, cannot create the ReActor");
            ReActorT actor = (*self->reactor_factory_)();
            while (!self->abort_) {
                std::optional<MessageT> message = self->message_queue_.blocking_pop_with(
                        [&] {
                            bool is_abort = self->abort_;
                            return is_abort;
                        },
                        self->GetTimeoutMS());
                if (!message) {
                    if (!self->abort_) {
                        // Log in case of failures ?
                        SLAM_LOG(INFO)
                                << "[EventLoopReActorWrapper] Timout exceeded, no message was returned, exiting. "
                                << std::endl;
                        self->abort_ = true;
                    }
                    self->is_running = false;
                    return;
                }
                actor.template React<MessageT>(*message);

            }
            self->is_running = false;
            self->abort_ = false;
        }

        Handler() {
            if constexpr(std::is_default_constructible_v<ReActorT>) {
                reactor_factory_ = std::move(MakeTrivialFactory<ReActorT>());
            }
        };

        explicit Handler(ReactorFactoryPtr<ReActorT> &&factory) {
            reactor_factory_ = std::move(factory);
        };

        // Pushes a message to the queue of the EventLoop
        void PushMessage(MessageT message) {
            message_queue_.push(message);
        }

        // Whether the Event Loop is already running
        bool IsRunning() const;

        void Start();

        // Sets the factory which creates the ReActor in the Event Loop
        void SetFactory(ReactorFactoryPtr<ReActorT> &&reactor);

        // Returns a reference to the message queue to be accessed and modified concurrently safely
        REF_GETTER(ConcurrentQueue, message_queue_);

        // The timeout for the queue in the main event loop,
        GETTER_SETTER(TimeoutMS, timeout_ms_, size_t);

        // Aborts the Event Loop
        void Abort();

    private:
        friend class std::thread;

        std::unique_ptr<std::thread> thread_;
        size_t timeout_ms_ = slam::kInvalidIndex;
        mutable std::mutex factory_mutex;
        ReactorFactoryPtr<ReActorT> reactor_factory_ = nullptr;
        std::atomic<bool> abort_ = false;
        std::atomic<bool> is_running = false;
        slam::blocking_queue <MessageT> message_queue_;
    };


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// IMPLEMENTATIONS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    template<typename ReActorT, typename MessageT, typename _>
    bool Handler<ReActorT, MessageT, _>::IsRunning() const {
        std::lock_guard<std::mutex> lock{factory_mutex};
        return is_running;
    }

    /* -------------------------------------------------------------------------------------------------------------- */
    template<typename ReActorT, typename MessageT, typename _>
    void
    Handler<ReActorT, MessageT, _>::SetFactory(ReactorFactoryPtr<ReActorT>
                                               &&reactor) {
        std::lock_guard<std::mutex> lock{factory_mutex};
        SLAM_CHECK_STREAM(!is_running, "The ReActor is already Running");
        reactor_factory_ = std::move(reactor);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    template<typename ReActorT, typename MessageT, typename _>
    void Handler<ReActorT, MessageT, _>::Start() {
        {
            std::lock_guard<std::mutex> lock{factory_mutex};
            SLAM_CHECK_STREAM(!is_running, "The Event Loop is already running !")
            SLAM_CHECK_STREAM(!thread_, "The thead has already been created !")
        }
        thread_ = std::make_unique<std::thread>(Run, this);
    }

/* -------------------------------------------------------------------------------------------------------------- */
    template<typename ReActorT, typename MessageT, typename _>
    void Handler<ReActorT, MessageT, _>::Abort() {
        abort_ = true;
        message_queue_.notify_event();
        if (thread_) {
            thread_->join();
            thread_ = nullptr;
        }
        abort_ = false;
    }
/* -------------------------------------------------------------------------------------------------------------- */

} // namespace slam

#endif //SLAMCORE_HANDLER_H
