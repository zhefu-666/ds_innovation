#include "rescue/controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace rescue {

Controller::Controller(Config config)
    : config_(std::move(config)) {
    initUART(config_.uart_port, config_.baudrate, config_.dry_run, config_.auto_run);
    initializeState();
}

Controller::~Controller() {
    safeShutdown();
}

void Controller::initializeState() {
    stop();
    release();
    sleepInterruptible(Ms(300));
}

void Controller::safeShutdown() {
    if (cleanup_done_.exchange(true)) {
        return;
    }
    std::cout << "\nRunning safe shutdown...\n";
    try {
        stop();
        sleepInterruptible(Ms(100));
        release();
        sleepInterruptible(Ms(100));
    } catch (const std::exception &e) {
        std::cerr << "[WARN] Safe shutdown error: " << e.what() << "\n";
    }
    closePort();
    std::cout << "Hardware reset complete\n";
}

bool Controller::executeWithRetry() {
    constexpr int max_retries = 3;
    constexpr auto retry_delay = Ms(500);

    if (execute()) {
        return true;
    }

    for (int attempt = 0; attempt < max_retries && !g_should_exit.load(); ++attempt) {
        try {
            sleepInterruptible(retry_delay);
            reopenPort();
            if (execute()) {
                std::cout << "[INFO] Reconnected on attempt " << (attempt + 1) << "\n";
                return true;
            }
        } catch (const std::exception &e) {
            std::cerr << "[WARN] Reconnect attempt " << (attempt + 1)
                      << " failed: " << e.what() << "\n";
        }
    }

    std::cerr << "[ERR] Max UART retries reached\n";
    return false;
}

void Controller::forward(int speed) {
    setMotorSpeed(1, -speed);
    setMotorSpeed(2, -speed);
    executeWithRetry();
}

void Controller::backward(int speed) {
    setMotorSpeed(1, speed);
    setMotorSpeed(2, speed);
    executeWithRetry();
}

void Controller::left(int speed) {
    setMotorSpeed(1, -speed);
    setMotorSpeed(2, speed);
    executeWithRetry();
}

void Controller::right(int speed) {
    setMotorSpeed(1, speed);
    setMotorSpeed(2, -speed);
    executeWithRetry();
}

void Controller::stop() {
    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
    executeWithRetry();
}

void Controller::catchBall(int angle1, int angle2) {
    if (angle1 < 0) {
        angle1 = config_.catch_angle[0];
    }
    if (angle2 < 0) {
        angle2 = config_.catch_angle[1];
    }
    setServoAngle(1, angle1);
    setServoAngle(2, angle2);
    executeWithRetry();
    sleepInterruptible(Ms(300));
}

void Controller::release(int angle1, int angle2) {
    if (angle1 < 0) {
        angle1 = config_.release_angle[0];
    }
    if (angle2 < 0) {
        angle2 = config_.release_angle[1];
    }
    setServoAngle(1, angle1);
    setServoAngle(2, angle2);
    executeWithRetry();
    sleepInterruptible(Ms(300));
}

void Controller::startCatch() {
    setServoAngle(1, 90);
    executeWithRetry();
    sleepInterruptible(Ms(300));
    setServoAngle(1, 0);
    executeWithRetry();
    sleepInterruptible(Ms(300));
}

void Controller::searchBall(int speed) {
    left(speed);
}

void Controller::searchArea(int speed) {
    right(speed);
}

void Controller::searchCross(int speed) {
    left(speed);
}

void Controller::approachBall(float x, float y) {
    approachTarget(x, y, 0.15f, true);
}

void Controller::approachArea(float x, float y) {
    approachTarget(x, y, 0.12f, false);
}

void Controller::approachTarget(float x, float y, float kp, bool print_debug) {
    constexpr float ki = 0.001f;
    constexpr float kd = 0.03f;
    constexpr int min_speed = 400;
    const float frame_height = static_cast<float>(config_.frame_height);
    const float center_x = static_cast<float>(config_.frame_width) * 0.5f;
    const float error_x = x - center_x;

    float base_speed = 5.5f;
    float speed_factor = 1.0f;
    if (y > frame_height * 0.75f) {
        base_speed = 4.0f;
        speed_factor = 0.5f;
    } else if (y > frame_height * 0.5f) {
        base_speed = 4.0f;
        speed_factor = 0.7f;
    }

    const auto now = Clock::now();
    float dt = std::chrono::duration<float>(now - pid_last_time_).count();
    if (dt <= 0.0001f || dt > 1.0f) {
        dt = 0.01f;
    }
    pid_last_time_ = now;

    const float p = kp * error_x;
    pid_integral_ += error_x * dt;
    pid_integral_ = std::clamp(pid_integral_, -100.0f, 100.0f);
    const float i = ki * pid_integral_;
    const float derivative = (error_x - pid_last_error_) / dt;
    const float d = kd * derivative;
    pid_last_error_ = error_x;

    const float pid_output = p + i + d;
    const float max_correction = base_speed * 0.8f;
    const float correction = std::clamp(pid_output, -max_correction, max_correction);

    float left_speed = (base_speed + correction) * speed_factor;
    float right_speed = (base_speed - correction) * speed_factor;

    int left_formatted = static_cast<int>(left_speed * 100.0f);
    int right_formatted = static_cast<int>(right_speed * 100.0f);
    if (std::abs(left_formatted) < min_speed) {
        left_formatted = left_formatted >= 0 ? min_speed : -min_speed;
    }
    if (std::abs(right_formatted) < min_speed) {
        right_formatted = right_formatted >= 0 ? min_speed : -min_speed;
    }

    setMotorSpeed(1, -left_formatted);
    setMotorSpeed(2, -right_formatted);

    if (print_debug) {
        if (std::abs(pid_output) > std::numeric_limits<float>::epsilon()) {
            std::cout << "PID correction: " << correction << " (P:" << p
                      << " I:" << i << " D:" << d << ")\n";
        }
        std::cout << "Ball position: (" << x << ", " << y << ") | left: "
                  << -left_formatted << " | right: " << -right_formatted << "\n";
    }

    executeWithRetry();
}

} // namespace rescue
