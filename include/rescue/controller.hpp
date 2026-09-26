#pragma once

#include "rescue/config.hpp"
#include "rescue/uart_controller.hpp"
#include "rescue/utils.hpp"

#include <atomic>

namespace rescue {

class Controller : public UARTController {
public:
    explicit Controller(Config config);
    ~Controller() override;

    void initializeState();
    void safeShutdown();
    bool executeWithRetry();

    void forward(int speed);
    void backward(int speed);
    void left(int speed);
    void right(int speed);
    void stop();

    void catchBall(int angle1 = -1, int angle2 = -1);
    void release(int angle1 = -1, int angle2 = -1);
    void startCatch();

    void searchBall(int speed);
    void searchArea(int speed);
    void searchCross(int speed);

    void approachBall(float x, float y);
    void approachArea(float x, float y);

private:
    void approachTarget(float x, float y, float kp, bool print_debug);

    Config config_;
    std::atomic_bool cleanup_done_{false};
    float pid_integral_ = 0.0f;
    float pid_last_error_ = 0.0f;
    Clock::time_point pid_last_time_ = Clock::now();
};

} // namespace rescue
