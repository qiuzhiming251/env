#include <iostream>
#include <chrono>
#include <string>
#include <functional>
namespace cem {
namespace fusion {

class FunctionTimer {
public:
    FunctionTimer(const std::string& function_name) : function_name_(function_name) {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    ~FunctionTimer() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);
        if (duration.count() > 5) {
            AINFO << "Function " << function_name_ << " took " << duration.count() << " ms";
        }
    }

    template <typename Func, typename... Args>
    static void measureTime(const std::string& function_name, Func func, Args... args) {
        FunctionTimer timer(function_name);
        func(args...);
    }

private:
    std::string function_name_;
    std::chrono::high_resolution_clock::time_point start_time_;
};

} // namespace fusion
} // namespace cem