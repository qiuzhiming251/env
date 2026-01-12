
#ifndef TIMECOSTTOOL_H
#define TIMECOSTTOOL_H

#include <memory>
#include <mutex>
#include <chrono>
#include <iostream>
#include <map>
#include <thread>
#include <unistd.h>

namespace cem {
    namespace fusion {

    class TimeCost {
    public:
        TimeCost() {
            startTime = std::chrono::high_resolution_clock::now();
        }

        inline long GetTimeCost() {
            auto end = std::chrono::high_resolution_clock::now();
            return  std::chrono::duration_cast<std::chrono::microseconds>(end - startTime).count();
        };

    private:
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    };


    class TimeCostTool {

    public:

        static TimeCostTool* getInstance();

    private:
        TimeCostTool();
        static TimeCostTool* instance;
        static std::mutex mutex_;

        std::map<std::string, TimeCost*>  TimeCostCasePool;
        std::mutex mutex_case_pool_;

    public:

        inline bool InitCase(const std::string& case_name) {
            if(case_name.empty()) {
                return false;
            }
            std::lock_guard<std::mutex> lock(mutex_case_pool_);
            if(TimeCostCasePool.find(case_name) == TimeCostCasePool.end()) {
                TimeCostCasePool.insert(std::pair<std::string, TimeCost*>(case_name, new TimeCost()));
                return true;
            }else {
                return false;
            }
        }

        inline long ReleaseCase(const std::string& case_name) {
            std::lock_guard<std::mutex> lock(mutex_case_pool_);
            auto it = TimeCostCasePool.find(case_name);
            if(it != TimeCostCasePool.end()) {
                long timeCost = it->second->GetTimeCost();
                std::cout <<"[TimeCostTool] " << case_name <<": " << timeCost << "us"<< std::endl;
                delete it->second;
                TimeCostCasePool.erase(it);
                return timeCost;
            }
            return 0;
        }
    };

    }
}

#define TIMECOSTSTART(...) \
    cem::fusion::TimeCostTool::getInstance()->InitCase(#__VA_ARGS__);

#define TIMECOSTEND(...) \
    cem::fusion::TimeCostTool::getInstance()->ReleaseCase(#__VA_ARGS__);

#endif //TIMECOSTTOOL_H
