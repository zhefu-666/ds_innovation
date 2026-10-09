#include "rescue/async_log.hpp"
#include <iostream>
#include <stdexcept>
int main() {
    // Saturated pipe models a stalled SSH/tee reader. Never drain it.
    int p[2]; if (pipe(p)) return 1;
    auto start = std::chrono::steady_clock::now();
    {
        rescue::AsyncLog log(p[1]);
        for (int i=0; i<20000; ++i) log.submit(std::string(4096, 'x'));
        if (!log.dropped()) throw std::runtime_error("queue was not bounded");
    }
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
    if (ms > 1000) throw std::runtime_error("blocked sink stalled producer/shutdown");
    close(p[0]); close(p[1]);
    // A disconnected reader must not SIGPIPE-terminate the controller.
    if (pipe(p)) return 1;
    close(p[0]);
    { rescue::AsyncLog log(p[1]); log.submit("closed reader\n");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (!log.dropped()) throw std::runtime_error("sink failure not counted"); }
    close(p[1]);
    // Verify the asynchronous path actually writes when the sink is healthy.
    if (pipe(p)) return 1;
    { rescue::AsyncLog log(p[1]); log.submit("healthy\n");
      pollfd ready{p[0], POLLIN, 0};
      if (poll(&ready, 1, 500) != 1) throw std::runtime_error("no log delivered");
      char buf[8]; if (read(p[0], buf, 8) != 8 || std::string(buf, 8) != "healthy\n") return 1; }
    close(p[0]); close(p[1]);
    std::cout << "async_log_tests passed; blocked producer/shutdown ms=" << ms << "\n";
}
