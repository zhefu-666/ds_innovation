#include "rescue/utils.hpp"

#include <algorithm>
#include <thread>

namespace rescue {

std::atomic_bool g_should_exit{false};

void signalHandler(int) {
    g_should_exit.store(true);
}

void sleepInterruptible(Ms duration) {
    const auto end = Clock::now() + duration;
    while (!g_should_exit.load() && Clock::now() < end) {
        const auto left = std::chrono::duration_cast<Ms>(end - Clock::now());
        std::this_thread::sleep_for(std::min(left, Ms(20)));
    }
}

} // namespace rescue
