#pragma once
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <streambuf>
#include <string>
#include <time.h>
#include <vector>

namespace rescue {
inline bool auditQuietLogs = false;
inline double auditWallMs() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
inline double auditCpuMs() {
    timespec t{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
    return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}
class StageAudit {
    struct Samples {
        std::vector<double> wall, cpu;
    };
    std::string path_;
    std::mutex mutex_;
    std::map<std::string, Samples> samples_;
    double started_ = auditWallMs();

  public:
    StageAudit() {
        if (const char* p = std::getenv("RESCUE_STAGE_AUDIT"))
            path_ = p;
    }
    static StageAudit& instance() {
        static StageAudit p;
        return p;
    }
    bool enabled() const {
        return !path_.empty();
    }
    void add(const std::string& name, double wall, double cpu) noexcept {
        try {
            std::lock_guard<std::mutex> g(mutex_);
            auto& s = samples_[name];
            s.wall.push_back(wall);
            s.cpu.push_back(cpu);
        } catch (...) {
        }
    }
    ~StageAudit() {
        try {
            if (!enabled())
                return;
            std::ofstream out(path_);
            out << "# elapsed_s=" << (auditWallMs() - started_) / 1000.0 << "\n";
            out << "stage,count,wall_mean_ms,wall_p50_ms,wall_p95_ms,cpu_mean_ms,wall_total_ms,cpu_"
                   "total_ms\n";
            for (auto& entry : samples_) {
                auto& s = entry.second;
                const size_t begin = std::min<size_t>(5, s.wall.size());
                if (begin == s.wall.size())
                    continue;
                std::vector<double> w(s.wall.begin() + begin, s.wall.end());
                double ws = 0, cs = 0;
                for (size_t i = begin; i < s.wall.size(); ++i) {
                    ws += s.wall[i];
                    cs += s.cpu[i];
                }
                std::sort(w.begin(), w.end());
                out << entry.first << ',' << w.size() << ',' << ws / w.size() << ','
                    << w[w.size() / 2] << ',' << w[(w.size() - 1) * 95 / 100] << ','
                    << cs / w.size() << ',' << ws << ',' << cs << '\n';
            }
        } catch (...) {
        }
    }
};
class AuditScope {
    std::string name_;
    double wall_ = 0, cpu_ = 0;
    bool active_ = false;

  public:
    explicit AuditScope(const std::string& name) {
        active_ = StageAudit::instance().enabled();
        if (active_) {
            name_ = name;
            wall_ = auditWallMs();
            cpu_ = auditCpuMs();
        }
    }
    void finish() noexcept {
        if (active_) {
            const double wall = auditWallMs() - wall_, cpu = auditCpuMs() - cpu_;
            active_ = false;
            StageAudit::instance().add(name_, wall, cpu);
        }
    }
    ~AuditScope() {
        finish();
    }
};
class AuditQuietCout {
    class NullBuffer : public std::streambuf {
        int_type overflow(int_type c) override {
            return traits_type::not_eof(c);
        }
        std::streamsize xsputn(const char*, std::streamsize n) override {
            return n;
        }
    } sink_;
    std::streambuf* old_ = nullptr;

  public:
    explicit AuditQuietCout(bool dry) {
        if (dry && StageAudit::instance().enabled() && std::getenv("RESCUE_AUDIT_QUIET")) {
            auditQuietLogs = true;
            old_ = std::cout.rdbuf(&sink_);
        }
    }
    ~AuditQuietCout() {
        if (old_)
            std::cout.rdbuf(old_);
        auditQuietLogs = false;
    }
};
} // namespace rescue
