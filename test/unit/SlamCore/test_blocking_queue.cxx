#include <thread>

#include <gtest/gtest.h>

#include <SlamCore/concurrent/blocking_queue.h>


using namespace std::chrono_literals;
/* ------------------------------------------------------------------------------------------------------------------ */
// Test the thread safety of the blocking queue

void RunConsumerThread(slam::blocking_queue<int> *queue) {
    int last_item = -1;
    while (last_item < 99) {
        auto new_value = queue->blocking_pop();
        ASSERT_TRUE(new_value.has_value());
        last_item = *new_value;
    }
    ASSERT_TRUE(last_item == 99);
}

void RunProducerThread(slam::blocking_queue<int> *queue) {
    for (int i(0); i < 100; i++) {
        queue->push(i);
        std::this_thread::sleep_for(10ms);
    }
}


TEST(blocking_queue, thread_safe) {

    slam::blocking_queue<int> queue;
    std::thread consumer{RunConsumerThread, &queue};
    std::this_thread::sleep_for(100ms);
    std::thread producer{RunProducerThread, &queue};

    consumer.join();
    producer.join();
}