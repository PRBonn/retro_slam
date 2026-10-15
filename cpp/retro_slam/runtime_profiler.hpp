// runtime_profiler.hpp - header-only runtime profiling utility
// Dumps summary JSON to ./runtime_profile.json on program exit

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace retro_slam {
namespace profiler {

struct SampleStats {
    size_t count = 0;
    double total_ns = 0.0;
    double min_ns = 0.0;
    double max_ns = 0.0;
    double mean_ns = 0.0;
    double stddev_ns = 0.0;
};

class RuntimeProfiler {
public:
    static RuntimeProfiler &Instance() {
        static RuntimeProfiler instance;
        return instance;
    }

    void RecordSample(const std::string &name, double duration_ns) {
        std::lock_guard<std::mutex> lock(mu_);
        samples_[name].push_back(duration_ns);
    }

    SampleStats ComputeStats(const std::vector<double> &vals) const {
        SampleStats s;
        if (vals.empty()) return s;
        s.count = vals.size();
        s.total_ns = std::accumulate(vals.begin(), vals.end(), 0.0);
        s.min_ns = *std::min_element(vals.begin(), vals.end());
        s.max_ns = *std::max_element(vals.begin(), vals.end());
        s.mean_ns = s.total_ns / static_cast<double>(s.count);
        double var = 0.0;
        for (double v : vals) var += (v - s.mean_ns) * (v - s.mean_ns);
        if (s.count > 1) var /= static_cast<double>(s.count - 1);
        s.stddev_ns = std::sqrt(var);
        return s;
    }

    void DumpJson(const std::string &path) {
        std::lock_guard<std::mutex> lock(mu_);
        std::cout << "dumping json to " << path << std::endl;
        std::ofstream ofs(path);
        if (!ofs.is_open()) return;
        ofs << "{\n";
        bool first = true;
        for (const auto &kv : samples_) {
            if (!first) ofs << ",\n";
            first = false;
            const auto &name = kv.first;
            const auto &vals = kv.second;
            const SampleStats s = ComputeStats(vals);
            ofs << "  \"" << EscapeString(name) << "\": {\n";
            ofs << "    \"count\": " << s.count << ",\n";
            ofs << "    \"total_ms\": " << Fixed(s.total_ns / 1e6) << ",\n";
            ofs << "    \"mean_ms\": " << Fixed(s.mean_ns / 1e6) << ",\n";
            ofs << "    \"min_ms\": " << Fixed(s.min_ns / 1e6) << ",\n";
            ofs << "    \"max_ms\": " << Fixed(s.max_ns / 1e6) << ",\n";
            ofs << "    \"stddev_ms\": " << Fixed(s.stddev_ns / 1e6) << "\n";
            ofs << "  }";
        }
        ofs << "\n}\n";
    }

    // explicit flush (optional)
    void FlushToDefault() { DumpJson(default_path_); }

    void SetDefaultPath(const std::string &p) { default_path_ = p; }

private:
    RuntimeProfiler() { /* nothing */
    }
    ~RuntimeProfiler() { DumpJson(default_path_); }
    RuntimeProfiler(const RuntimeProfiler &) = delete;
    RuntimeProfiler &operator=(const RuntimeProfiler &) = delete;

    static std::string EscapeString(const std::string &s) {
        std::ostringstream oss;
        for (char c : s) {
            switch (c) {
                case '"':
                    oss << "\\\"";
                    break;
                case '\\':
                    oss << "\\\\";
                    break;
                case '\n':
                    oss << "\\n";
                    break;
                case '\r':
                    oss << "\\r";
                    break;
                case '\t':
                    oss << "\\t";
                    break;
                default:
                    oss << c;
            }
        }
        return oss.str();
    }

    static std::string Fixed(double v) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << v;
        return oss.str();
    }

    mutable std::mutex mu_;
    std::map<std::string, std::vector<double>> samples_;
    std::string default_path_ = "./runtime_profile.json";
};

// RAII scoped timer
struct ScopedTimer {
    explicit ScopedTimer(const std::string &name)
        : name_(name), start_(std::chrono::steady_clock::now()) {}
    void end() {
        const auto end = std::chrono::steady_clock::now();
        const double ns = std::chrono::duration<double, std::nano>(end - start_).count();
        RuntimeProfiler::Instance().RecordSample(name_, ns);
    }
    std::string name_;
    std::chrono::steady_clock::time_point start_;
};

}  // namespace profiler
}  // namespace retro_slam
