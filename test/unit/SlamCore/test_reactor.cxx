#include <gtest/gtest.h>
#include <SlamCore/reactors/reactor.h>
#include <SlamCore/reactors/handler.h>

/* ------------------------------------------------------------------------------------------------------------------ */
// A Simple Test Actor
template<>
struct slam::message_tag<class PrintMessageReActor, const std::string *> {
    constexpr static bool is_valid = true;
};

class PrintMessageReActor : public slam::GenericReactor<PrintMessageReActor> {
public:
    typedef const std::string *message_str_t;

    void DoReact(message_str_t message,
                 slam::message_tag<PrintMessageReActor, message_str_t>) {
        std::cout << "Received message: " << *message << std::endl;
    }
};


/* ------------------------------------------------------------------------------------------------------------------ */
// Test showing the API of an actor
TEST(Reactor, test) {
    PrintMessageReActor actor;
    const std::string message = "Hello World";
    actor.React(&message);
    //^ Because message_tag<PrintMessageActor, message_str_t>::is_valid is true,
    //  the compiler does find DoReact(., message_str_t)

//    int invalid_message = 42;
//    actor.React(invalid_message);
//^ Does not compile because message_tag<PrintMessageActor, const std::string*>::is_valid is false
}

/* ------------------------------------------------------------------------------------------------------------------ */
// Test showing the EventLoop in action
TEST(Reactor, EventLoop) {
    using namespace std::chrono_literals;
    slam::Handler<PrintMessageReActor,
            PrintMessageReActor::message_str_t> handler;

    handler.Start();
    std::string hello_world = "Hello Actor World !!";
    for (auto i(0); i < 3; ++i) {
        handler.PushMessage(&hello_world);
        std::this_thread::sleep_for(10ms);
    }
    while (!handler.ConcurrentQueue().empty()) {
        std::this_thread::sleep_for(1ms);
    }
    handler.Abort();
}


