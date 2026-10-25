#include "rescue/app/runtime.hpp"
#include "rescue/core/utils.hpp"
#include <csignal>
#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    std::signal(SIGINT, rescue::signalHandler);
    std::signal(SIGTERM, rescue::signalHandler);
    try {
        return rescue::app::runApplication(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "[FATAL] " << error.what() << "\n";
        return 1;
    }
}
