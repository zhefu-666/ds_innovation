#pragma once

#include <atomic>
#include <chrono>

namespace rescue {

using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

extern std::atomic_bool g_should_exit;

void signalHandler(int);
void sleepInterruptible(Ms duration);

} // namespace rescue
