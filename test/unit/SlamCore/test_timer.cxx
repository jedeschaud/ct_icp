#include <gtest/gtest.h>
#include <SlamCore/timer.h>
#include <thread>
#include <chrono>
#include <iostream>

TEST(test_timer, basics) {

    slam::Timer timer;
    using tick = slam::Timer::Ticker;
    using namespace std::chrono_literals;

    {
        tick ticker1(timer, "entry1");
        std::this_thread::sleep_for(20ms);
        tick ticker2(timer, "entry2");
        std::this_thread::sleep_for(20ms);
    }
    {
        tick ticker1(timer, "entry1");
        std::this_thread::sleep_for(20ms);
        tick ticker2(timer, "entry2");
        std::this_thread::sleep_for(20ms);
    }
    {
        tick ticker2(timer, "entry1");
        std::this_thread::sleep_for(20ms);
        tick ticker1(timer, "entry2");
        std::this_thread::sleep_for(20ms);
    }


    timer.WriteMessage(std::cout);


}

