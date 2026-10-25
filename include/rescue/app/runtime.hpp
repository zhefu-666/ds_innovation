#pragma once

namespace rescue {
struct ResolvedRuntimeConfig;
namespace app {

class Runtime {
  public:
    explicit Runtime(const ResolvedRuntimeConfig& config) : config_(config) {
    }
    int run();

  private:
    const ResolvedRuntimeConfig& config_;
};

int runApplication(int argc, char** argv);
} // namespace app
} // namespace rescue
